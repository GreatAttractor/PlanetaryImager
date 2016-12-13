/*
 * Copyright (C) 2016  Marco Gulino <marco@gulinux.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "zwo_asi_imager.h"
#include "drivers/imagerthread.h"
#include <stringbuilder.h>
#include <QObject>
#include <set>
#include <QThread>
#include "commons/fps_counter.h"
#include <QRect>
#include <atomic>
#include <ratio>
#include "Qt/strings.h"
#include "commons/utils.h"
#include "zwoexception.h"
#include "asiimagingworker.h"
#include <QTimer>
#include "asicontrol.h"
#include <QCoreApplication>
#include "drivers/roi.h"
#include "c++/stlutils.h"

using namespace std;
using namespace std::chrono_literals;
using namespace GuLinux;


namespace {
const int64_t ImgTypeControlID = 10000;
const int64_t BinControlID = 10001;
}

Q_DECLARE_METATYPE(ASI_IMG_TYPE)

DPTR_IMPL(ZWO_ASI_Imager) {
    ASI_CAMERA_INFO info;
    shared_ptr<QTimer> reload_temperature_timer;
    ZWO_ASI_Imager *q;

    Properties properties;

    ASIControl::vector controls;
    ASIControl::ptr temperature_control;
    
    ASIImagingWorker::ptr worker;
    ROIValidator::ptr roi_validator;
    QRect maxROI(int bin) const;
    void restart_worker(int bin, const QRect &roi, ASI_IMG_TYPE format);
    void read_temperature();
    void update_worker_exposure_timeout();
};

void ZWO_ASI_Imager::Private::read_temperature() {
  qDebug() << "Refreshing ASI_TEMPERATURE if found..";
  if(temperature_control)
    q->push_job_on_thread([=]{
      emit q->temperature(temperature_control->reload().control().value.toDouble());
    });
}


ZWO_ASI_Imager::ZWO_ASI_Imager(const ASI_CAMERA_INFO &info, const ImageHandler::ptr &imageHandler) : Imager{imageHandler}, dptr(info, make_shared<QTimer>(), this)
{
    d->roi_validator = make_shared<ROIValidator>(initializer_list<ROIValidator::Rule>{
      ROIValidator::x_multiple(2),
      ROIValidator::y_multiple(2),
      ROIValidator::width_multiple(8),
      ROIValidator::height_multiple(2),
      [=](QRect &roi) {
        if(info.IsUSB3Camera == ASI_FALSE && QString{info.Name}.contains("120")) {
          ROIValidator::area_multiple(1024, 0, 2, QRect{0, 0, static_cast<int>(info.MaxWidth), static_cast<int>(info.MaxHeight)})(roi);
          qDebug() << "Using ASI 120MM rect roi definition: " << roi;
        }
      }
    });
    d->properties.set_resolution_pixelsize({static_cast<int>(info.MaxWidth), static_cast<int>(info.MaxHeight)}, info.PixelSize, info.PixelSize);
    d->properties << Properties::Property{"Camera Speed", info.IsUSB3Camera ? "USB3" : "USB2"}
                  << Properties::Property{"Host Speed", info.IsUSB3Host ? "USB3" : "USB2"}
                  << Properties::Property{"Camera Type", info.IsColorCam ? "colour" : "mono"};
    if(info.IsColorCam) {
        static map<ASI_BAYER_PATTERN, QString> patterns {
            {ASI_BAYER_RG, "RGGB"}, {ASI_BAYER_BG, "BGGR"}, {ASI_BAYER_GR, "GRBG"}, {ASI_BAYER_GB, "GBRG"}
        };
        d->properties << Properties::Property{"Bayer pattern", patterns[info.BayerPattern]};
    }
    
    d->properties << Properties::Property{"ElecPerADU", info.ElecPerADU};
    d->properties << Properties::Property{"ASI SDK Version", ASI_SDK_VERSION};
    d->properties << LiveStream << ROI << Temperature;
    ASI_CHECK << ASIOpenCamera(info.CameraID) << "Open Camera";
#ifdef ASI_CAMERA_REQUIRES_INIT
    ASI_CHECK << ASIInitCamera(info.CameraID) << "Init Camera";
#endif
    connect(d->reload_temperature_timer.get(), &QTimer::timeout, this, bind(&Private::read_temperature, d.get() ));
    d->reload_temperature_timer->start(5000);
    connect(this, &Imager::exposure_changed, this, bind(&Private::update_worker_exposure_timeout, d.get()));
}

ZWO_ASI_Imager::~ZWO_ASI_Imager()
{
    d->reload_temperature_timer->stop();
    ASI_CHECK << ASICloseCamera(d->info.CameraID) << "Close Camera";
}

Imager::Properties ZWO_ASI_Imager::properties() const
{
    return d->properties;
}

QString ZWO_ASI_Imager::name() const
{
    return d->info.Name;
}

void ZWO_ASI_Imager::Private::update_worker_exposure_timeout()
{
  if(worker)
    worker->calc_exposure_timeout();
}


Imager::Controls ZWO_ASI_Imager::controls() const
{
    Controls controls;
    int controls_number;
    ASI_CHECK << ASIGetNumOfControls(d->info.CameraID, &controls_number) << "Get controls";
    d->controls = ASIControl::vector(controls_number);

    for(int control_index = 0; control_index < controls_number; control_index++) {
      auto control = make_shared<ASIControl>(control_index, d->info.CameraID);
      d->controls[control_index] = control;
      if(control->caps.ControlType == ASI_TEMPERATURE)
        d->temperature_control = control;
      else
        controls.push_back(control->control());
    }

    static map<ASI_IMG_TYPE, QString> format_names {
        {ASI_IMG_RAW8, "Raw 8bit"},
        {ASI_IMG_RGB24, "RGB24"},
        {ASI_IMG_RAW16, "RAW 16bit"},
        {ASI_IMG_Y8, "Y8 (Bayer)"},
    };
    auto imageFormat = Control{ImgTypeControlID, "Image Format", Control::Combo}.set_value(d->worker->format());
    int i = 0;
    while(d->info.SupportedVideoFormat[i] != ASI_IMG_END && i < 8) {
        auto format = d->info.SupportedVideoFormat[i];
        qDebug() << "supported format: " << format << ": " << format_names[format];
        imageFormat.add_choice(format_names[format], format);
        ++i;
    }
    controls.push_front(imageFormat);

    auto bin = Control{BinControlID, "Bin", Control::Combo}.set_value(d->worker->bin());
    i = 0;
    while(d->info.SupportedBins[i] != 0) {
        auto bin_value = d->info.SupportedBins[i++];
        bin.add_choice("%1x%1"_q % bin_value, bin_value);
    }
    controls.push_front(bin);
    return controls;
}


void ZWO_ASI_Imager::setControl(const Control& control)
{
  LOG_F_SCOPE
  if(control.id == ImgTypeControlID) {
    d->restart_worker(d->worker->bin(), d->worker->roi(), control.get_value<ASI_IMG_TYPE>());
    emit changed(control);
    return;
  }
  if(control.id == BinControlID) {
    auto bin =control.get_value<int>();
    d->restart_worker(bin, d->maxROI(bin), d->worker->format());
    emit changed(control);
    return;
  }
  auto camera_control_it = find_if(d->controls.begin(), d->controls.end(),
			  [&](const ASIControl::ptr &c){ return c->caps.ControlType == static_cast<ASI_CONTROL_TYPE>(control.id); });
  if(camera_control_it != d->controls.end()) {
    auto camera_control = *camera_control_it;
    qDebug() << "Changing control " << camera_control->control();
    wait_for(push_job_on_thread([=]{
      camera_control->set(control.get_value<qlonglong>(), control.value_auto);
      qDebug() << "Changed control " << camera_control->control();
      emit changed(*camera_control);
    }));
  }
}




void ZWO_ASI_Imager::Private::restart_worker(int bin, const QRect& roi, ASI_IMG_TYPE format)
{
  auto factory = [=] {
    return worker = make_shared<ASIImagingWorker>(roi, bin, info, format);
  };
  worker.reset();
  q->restart(factory);
}



void ZWO_ASI_Imager::startLive()
{
    LOG_F_SCOPE
    d->restart_worker(1, d->maxROI(1), d->info.SupportedVideoFormat[0]);
    qDebug() << "Live started correctly";
}


void ZWO_ASI_Imager::clearROI()
{
  d->restart_worker(d->worker->bin(), d->maxROI(d->worker->bin()), d->worker->format());
}

void ZWO_ASI_Imager::setROI(const QRect& roi)
{
  
  d->restart_worker(d->worker->bin(), d->roi_validator->validate(roi), d->worker->format());
}

QRect ZWO_ASI_Imager::Private::maxROI(int bin) const
{
    return roi_validator->validate({0, 0, static_cast<int>(info.MaxWidth) / bin, static_cast<int>(info.MaxHeight) / bin});
}


#include "zwo_asi_imager.moc"

