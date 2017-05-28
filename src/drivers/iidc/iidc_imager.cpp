/*
 * Copyright (C) 2017 Filip Szczerek <ga.software@yahoo.com>
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

#include <algorithm>
#include "commons/utils.h"
#include "iidc_deleters.h"
#include "iidc_exception.h"
#include "iidc_imager.h"
#include <map>
#include <QRect>
#include "Qt/strings.h"
#include <thread>


constexpr uint32_t NUM_DMA_BUFFERS = 4;

enum ControlID: qlonglong
{
    VideoMode = DC1394_FEATURE_MAX + 1
};

static std::map<dc1394color_coding_t, const char *> COLOR_CODING_NAME
{
    { DC1394_COLOR_CODING_MONO8,   "Mono 8-bit"           },
    { DC1394_COLOR_CODING_YUV411,  "YUV411"               },
    { DC1394_COLOR_CODING_YUV422,  "YUV422"               },
    { DC1394_COLOR_CODING_YUV444,  "YUV444"               },
    { DC1394_COLOR_CODING_RGB8,    "RGB 8-bit"            },
    { DC1394_COLOR_CODING_MONO16,  "Mono 16-bit"          },
    { DC1394_COLOR_CODING_RGB16,   "RGB 16-bit"           },
    { DC1394_COLOR_CODING_MONO16S, "Mono 16-bit (signed)" },
    { DC1394_COLOR_CODING_RGB16S,  "RGB 16-bit (signed)"  },
    { DC1394_COLOR_CODING_RAW8,    "RAW 8-bit"            },
    { DC1394_COLOR_CODING_RAW16,   "RAW 16-bit"           },
};

class IIDCImagerWorker;

DPTR_IMPL(IIDCImager)
{
    std::unique_ptr<dc1394camera_t, Deleters::camera> camera;

    dc1394video_modes_t videoModes;

    dc1394video_mode_t currentVidMode;

    Properties properties;

    std::shared_ptr<IIDCImagerWorker> worker;

    void updateWorkerExposureTimeout();

    LOG_C_SCOPE(IIDCImager);
  //ROIValidator::ptr roi_validator;
};

void IIDCImager::Private::updateWorkerExposureTimeout()
{
//TODO: implement this
}

class IIDCImagerWorker: public ImagerThread::Worker
{
    dc1394camera_t *camera;
    dc1394video_frame_t *nativeFrame; ///< The most recently captured frame

    struct
    {
        bool initialized; // If 'false', the remaining fields have not been set yet

        Frame::ColorFormat colorFormat;
        uint8_t bitsPerChannel;
        Frame::ByteOrder byteOrder;
    } frameInfo;

    /// If 'vidMode' supports a fixed set of framerates, sets the highest supported framerate
    void setHighestFramerate(dc1394video_mode_t vidMode);

    void initFrameInfo();

    LOG_C_SCOPE(IIDCImagerWorker);

public:

    IIDCImagerWorker(dc1394camera_t *_camera, dc1394video_mode_t vidMode);
  
    Frame::ptr shoot() override;

    void setROI(const QRect &roi);

    QRect ROI() const { return { }; }

    virtual ~IIDCImagerWorker();
};

//Q_DECLARE_METATYPE(IIDCImagerWorker::ImageType)

IIDCImager::IIDCImager(std::unique_ptr<dc1394camera_t, Deleters::camera> camera, const ImageHandler::ptr &handler)
: Imager(handler), dptr()
{
    d->camera = std::move(camera);

    IIDC_CHECK << dc1394_video_get_supported_modes(d->camera.get(), &d->videoModes)
               << "Get supported video modes";

    d->currentVidMode = d->videoModes.modes[0];

    connect(this, &Imager::exposure_changed, this, std::bind(&Private::updateWorkerExposureTimeout, d.get()));
}

IIDCImager::~IIDCImager()
{
}


Imager::Properties IIDCImager::properties() const
{
    //TODO: obtain from the highest-resolution video mode;    d->properties.set_resolution_pixelsize()

    return Imager::Properties() << LiveStream;
}

QString IIDCImager::name() const
{
    return "IIDC Imager";
}

void IIDCImager::setControl(const Imager::Control& setting)
{
}

void IIDCImager::readTemperature()
{
}

Imager::Controls IIDCImager::controls() const
{
    Controls controls;

    auto videoMode = Control{ ControlID::VideoMode, "Video Mode", Control::Combo };

    for (uint32_t i = 0; i < d->videoModes.num; i++)
    {
        const dc1394video_mode_t vidMode = d->videoModes.modes[i];

        uint32_t width, height;
        IIDC_CHECK << dc1394_get_image_size_from_video_mode(d->camera.get(), vidMode, &width, &height)
                   << "Get image size from video mode";

        dc1394color_coding_t coding;
        IIDC_CHECK << dc1394_get_color_coding_from_video_mode(d->camera.get(), vidMode, &coding)
                   << "Get color coding from video mode";

        // FIXME: does not compile if COLOR_CODING_NAME is 'const'; possibly a problem with _q or %
        QString modeName = "%1x%2 %3"_q % width % height % COLOR_CODING_NAME[coding];

        if (vidMode >= DC1394_VIDEO_MODE_FORMAT7_MIN &&
            vidMode <= DC1394_VIDEO_MODE_FORMAT7_MAX)
        {
            // Format7 modes may have additional capabilities:
            //   - max framerate different than the predefined ones in 'dc1394framerate_t' enum
            //   - ROI support
            //   - various modes of pixel binning
            //
            // Some cameras may report only Format7 modes.

            modeName += " (FMT7: %10)"_q % (vidMode - DC1394_VIDEO_MODE_FORMAT7_0);
        }

        videoMode.add_choice_enum(modeName, vidMode);
    }
    videoMode.set_value_enum(d->currentVidMode);

    controls.push_back(std::move(videoMode));

    return controls;
}

IIDCImagerWorker::IIDCImagerWorker(dc1394camera_t *_camera, dc1394video_mode_t vidMode)
: camera(_camera), nativeFrame(nullptr)
{
    frameInfo.initialized = false;

    IIDC_CHECK << dc1394_video_set_mode(camera, vidMode)
               << "Set video mode";

    setHighestFramerate(vidMode);

    IIDC_CHECK << dc1394_capture_setup(camera, NUM_DMA_BUFFERS, DC1394_CAPTURE_FLAGS_DEFAULT)
               << "Setup capture";

    IIDC_CHECK << dc1394_video_set_transmission(camera, DC1394_ON)
               << "Start video transmission";
}

IIDCImagerWorker::~IIDCImagerWorker()
{
    IIDC_CHECK << dc1394_video_set_transmission(camera, DC1394_OFF)
               << "Stop video transmission";

    IIDC_CHECK << dc1394_capture_stop(camera)
               << "Stop capture";
}

void IIDCImagerWorker::initFrameInfo()
{
    switch (nativeFrame->color_coding)
    {
    case DC1394_COLOR_CODING_MONO8:
    case DC1394_COLOR_CODING_MONO16:
    case DC1394_COLOR_CODING_MONO16S:
        frameInfo.colorFormat = Frame::ColorFormat::Mono; break;

    // YUV formats will be converted to RGB before returning the frame
    case DC1394_COLOR_CODING_YUV411:
    case DC1394_COLOR_CODING_YUV422:
    case DC1394_COLOR_CODING_YUV444:
    case DC1394_COLOR_CODING_RGB8:
    case DC1394_COLOR_CODING_RGB16:
    case DC1394_COLOR_CODING_RGB16S:
        frameInfo.colorFormat = Frame::ColorFormat::RGB; break;

    case DC1394_COLOR_CODING_RAW8:
    case DC1394_COLOR_CODING_RAW16:
        switch (nativeFrame->color_filter)
        {
        case DC1394_COLOR_FILTER_RGGB: frameInfo.colorFormat = Frame::ColorFormat::Bayer_RGGB; break;
        case DC1394_COLOR_FILTER_BGGR: frameInfo.colorFormat = Frame::ColorFormat::Bayer_BGGR; break;
        case DC1394_COLOR_FILTER_GBRG: frameInfo.colorFormat = Frame::ColorFormat::Bayer_GBRG; break;
        case DC1394_COLOR_FILTER_GRBG: frameInfo.colorFormat = Frame::ColorFormat::Bayer_GRBG; break;
        }
        break;
    }

    frameInfo.bitsPerChannel = nativeFrame->data_depth;
    frameInfo.byteOrder = (nativeFrame->little_endian ? Frame::ByteOrder::LittleEndian : Frame::ByteOrder::BigEndian);
}

Frame::ptr IIDCImagerWorker::shoot()
{
    //TODO: fail gracefully if cannot capture
    IIDC_CHECK << dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_WAIT, &nativeFrame)
               << "Capture dequeue";

    if (!frameInfo.initialized)
    {
        initFrameInfo();
        frameInfo.initialized = true;
    }

    auto frame = std::make_shared<Frame>(frameInfo.bitsPerChannel,
                                         frameInfo.colorFormat,
                                         QSize{ (int)nativeFrame->size[0], (int)nativeFrame->size[1] },
                                         frameInfo.byteOrder);

    //TODO: convert from YUV to RGB

    uint8_t *destLine = frame->mat().data;
    const size_t destLineStep = frame->mat().step[0];
    const size_t numCopyBytes = std::min(destLineStep, (size_t)nativeFrame->stride);

    for (int y = 0; y < frame->mat().rows; y++)
    {
        memcpy(destLine, nativeFrame->image + y * nativeFrame->stride, numCopyBytes);
        destLine += destLineStep;
    }

    IIDC_CHECK << dc1394_capture_enqueue(camera, nativeFrame)
               << "Capture enqueue";
    nativeFrame = nullptr;

    return frame;
}

void IIDCImager::clearROI()
{
}

void IIDCImager::setROI(const QRect &roi)
{
}

void IIDCImagerWorker::setROI(const QRect& roi)
{
}

void IIDCImagerWorker::setHighestFramerate(dc1394video_mode_t vidMode)
{
    if (!dc1394_is_video_mode_scalable(vidMode))
    {
        dc1394framerates_t framerates;
        IIDC_CHECK << dc1394_video_get_supported_framerates(camera, vidMode, &framerates)
                   << "Get supported framerates";

        dc1394framerate_t *highest = std::max_element(framerates.framerates, framerates.framerates + framerates.num);

        IIDC_CHECK << dc1394_video_set_framerate(camera, *highest)
                   << "Set framerate";
    }
}

void IIDCImager::startLive()
{
    restart([&] { return d->worker = std::make_shared<IIDCImagerWorker>(d->camera.get(), d->currentVidMode); });
    qDebug() << "Video streaming started successfully";
}
