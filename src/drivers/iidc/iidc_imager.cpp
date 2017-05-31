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

    dc1394featureset_t features;

    Properties properties;

    void updateWorkerExposureTimeout();

    Control enumerateVideoModes();

    LOG_C_SCOPE(IIDCImager);
  //ROIValidator::ptr roi_validator;
};

void IIDCImager::Private::updateWorkerExposureTimeout()
{
//TODO: implement this
}

Imager::Control IIDCImager::Private::enumerateVideoModes()
{
    auto videoMode = Imager::Control{ ControlID::VideoMode, "Video Mode", Control::Combo };

    for (uint32_t i = 0; i < videoModes.num; i++)
    {
        const dc1394video_mode_t vidMode = videoModes.modes[i];

        uint32_t width, height;
        IIDC_CHECK << dc1394_get_image_size_from_video_mode(camera.get(), vidMode, &width, &height)
                   << "Get image size from video mode";

        dc1394color_coding_t coding;
        IIDC_CHECK << dc1394_get_color_coding_from_video_mode(camera.get(), vidMode, &coding)
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

            modeName += " (FMT7: %1)"_q % (vidMode - DC1394_VIDEO_MODE_FORMAT7_0);
        }

        videoMode.add_choice_enum(modeName, vidMode);
    }
    videoMode.set_value_enum(currentVidMode);

    return videoMode;
}

class IIDCImagerWorker: public ImagerThread::Worker
{
    dc1394camera_t *camera;
    dc1394video_frame_t *nativeFrame; ///< The most recently captured frame

    struct
    {
        bool initialized; ///< If 'false', the remaining fields have not been set yet

        Frame::ColorFormat colorFormat;
        uint8_t bitsPerChannel;
        Frame::ByteOrder byteOrder;
        bool needsYUVtoRGBconversion;
        size_t srcBytesPerLine; ///< Set only for YUV formats
    } frameInfo;

    /// Used for YUV->RGB conversion
    /** Required, because conversion function expects buffers without line padding,
        which may be present in a dequeued capture buffer. */
    struct
    {
        std::unique_ptr<uint8_t[]> src, dest;
    } conversionBuf;

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

    IIDC_CHECK << dc1394_feature_get_all(d->camera.get(), &d->features)
               << "Get all features";

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

void IIDCImager::setControl(const Imager::Control& control)
{
    LOG_F_SCOPE

    if (control.id == ControlID::VideoMode)
    {
        d->currentVidMode = control.get_value_enum<dc1394video_mode_t>();
        startLive();

        emit changed(control);
    }
    else
    {
        if (control.supports_auto)
        {
            dc1394feature_mode_t mode;
            IIDC_CHECK << dc1394_feature_get_mode(d->camera.get(), (dc1394feature_t)control.id, &mode)
                       << "Get feature mode";

            if ((DC1394_FEATURE_MODE_AUTO == mode) ^ control.value_auto)
            {
                IIDC_CHECK << dc1394_feature_set_mode(d->camera.get(), (dc1394feature_t)control.id, control.value_auto ? DC1394_FEATURE_MODE_AUTO
                                                                                                                       : DC1394_FEATURE_MODE_MANUAL)
                           << "Set feature mode";
            }
        }

        if (control.supports_on_off)
        {
            dc1394switch_t onOffState;
            IIDC_CHECK << dc1394_feature_get_power(d->camera.get(), (dc1394feature_t)control.id, &onOffState)
                       << "Get feature on/off state";

            if ((DC1394_ON == onOffState) ^ control.value_on)
            {
                IIDC_CHECK << dc1394_feature_set_power(d->camera.get(), (dc1394feature_t)control.id, control.value_on ? DC1394_ON
                                                                                                                      : DC1394_OFF)
                           << "Set feature on/off state";
            }
        }
    }
}

void IIDCImager::readTemperature()
{
}

Imager::Controls IIDCImager::controls() const
{
    Controls controls;

    controls.push_back(std::move(d->enumerateVideoModes()));

    for (const dc1394feature_info_t &feature: d->features.feature)
        if (DC1394_TRUE == feature.available)
        {
            Control control{ feature.id };
            control.type = Control::Type::Number;

            switch (feature.id)
            {
                case DC1394_FEATURE_BRIGHTNESS:      control.name = "Brightness"; break;

                // This is not "exposure time" (see DC1394_FEATURE_SHUTTER for that);
                // instead, it regulates the desired overall image brightness by changing
                // values of shutter and gain - if they are set to auto
                case DC1394_FEATURE_EXPOSURE:        control.name = "Exposure"; break;

                case DC1394_FEATURE_SHARPNESS:       control.name = "Sharpness"; break;

                //TODO: this is in fact a pair of controls
                //case DC1394_FEATURE_WHITE_BALANCE:   control.name = "White balance"; break;

                case DC1394_FEATURE_HUE:             control.name = "Hue"; break;
                case DC1394_FEATURE_SATURATION:      control.name = "Saturation"; break;
                case DC1394_FEATURE_GAMMA:           control.name = "Gamma"; break;

                case DC1394_FEATURE_SHUTTER:
                    control.name = "Shutter";
                    control.is_exposure = true;
                    control.is_duration = true;
                    break;

                case DC1394_FEATURE_GAIN:            control.name = "Gain"; break;
                case DC1394_FEATURE_IRIS:            control.name = "Iris"; break;
                case DC1394_FEATURE_FOCUS:           control.name = "Focus"; break;
                case DC1394_FEATURE_TEMPERATURE:     control.name = "Temperature"; break;

                // TODO: requires special handling
                // case DC1394_FEATURE_TRIGGER:         control.name = "Trigger"; break;

                // TODO: uncomment when DC1394_FEATURE_TRIGGER is implemented
                // case DC1394_FEATURE_TRIGGER_DELAY:
                //     control.name = "Trigger delay";
                //     control.is_duration = true;
                //     break;

                //TODO: this is in fact a triple of controls
                //case DC1394_FEATURE_WHITE_SHADING:   control.name = "White shading"; break;

                case DC1394_FEATURE_FRAME_RATE:      control.name = "Frame rate"; break;
                case DC1394_FEATURE_ZOOM:            control.name = "Zoom"; break;
                case DC1394_FEATURE_PAN:             control.name = "Pan"; break;
                case DC1394_FEATURE_TILT:            control.name = "Tilt"; break;
                case DC1394_FEATURE_OPTICAL_FILTER:  control.name = "Optical filter"; break;
                case DC1394_FEATURE_CAPTURE_SIZE:    control.name = "Capture size"; break;
                case DC1394_FEATURE_CAPTURE_QUALITY: control.name = "Capture quality"; break;

                default: continue;
            }

            control.supports_auto = (std::find(feature.modes.modes, feature.modes.modes + feature.modes.num, DC1394_FEATURE_MODE_AUTO)
                                       != feature.modes.modes + feature.modes.num);

            control.readonly = (0 == feature.modes.num &&
                                DC1394_TRUE == feature.readout_capable);

            control.supports_on_off = (DC1394_TRUE == feature.on_off_capable);
            if (control.supports_on_off)
            {
                dc1394switch_t currOnOff;
                IIDC_CHECK << dc1394_feature_get_power(d->camera.get(), feature.id, &currOnOff)
                           << "Get feature on/off";
                control.value_on = (DC1394_ON == currOnOff);
            }

            dc1394feature_mode_t currMode;
            IIDC_CHECK << dc1394_feature_get_mode(d->camera.get(), feature.id, &currMode)
                        << "Get feature mode";
            control.value_auto = (DC1394_FEATURE_MODE_AUTO == currMode);

            dc1394_feature_set_absolute_control(d->camera.get(), feature.id, DC1394_OFF);

            controls.push_back(std::move(control));
        }


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

    switch (nativeFrame->color_coding)
    {
    case DC1394_COLOR_CODING_YUV411: frameInfo.srcBytesPerLine = (3 * nativeFrame->size[0] + 1) / 2; break;
    case DC1394_COLOR_CODING_YUV422: frameInfo.srcBytesPerLine = 2 * nativeFrame->size[0]; break;
    case DC1394_COLOR_CODING_YUV444: frameInfo.srcBytesPerLine = 3 * nativeFrame->size[0]; break;

    default: frameInfo.srcBytesPerLine = 0; break;
    }
}

static bool isYUV(dc1394color_coding_t colorCoding)
{
    return colorCoding == DC1394_COLOR_CODING_YUV411 ||
           colorCoding == DC1394_COLOR_CODING_YUV422 ||
           colorCoding == DC1394_COLOR_CODING_YUV444;
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

        if (isYUV(nativeFrame->color_coding))
        {
            frameInfo.needsYUVtoRGBconversion = true;
            conversionBuf.src = std::make_unique<uint8_t[]>(nativeFrame->total_bytes); // Pass 'total_bytes' for simplicity; we may use less
            conversionBuf.dest = std::make_unique<uint8_t[]>(nativeFrame->size[0] * nativeFrame->size[1] * 3); // 3 bytes/pixel (R, G, B)
        }
        else
            frameInfo.needsYUVtoRGBconversion = false;
    }

    const size_t imgWidth = nativeFrame->size[0],
                 imgHeight = nativeFrame->size[1];

    auto frame = std::make_shared<Frame>(frameInfo.bitsPerChannel,
                                         frameInfo.colorFormat,
                                         QSize{ (int)imgWidth, (int)imgHeight },
                                         frameInfo.byteOrder);

    uint8_t *srcLine;
    size_t srcLineStep;

    uint8_t *destLine = frame->mat().data;
    const size_t destLineStep = frame->mat().step[0];
    size_t numDestCopyBytes; // Number of bytes per line to copy into 'frame'

    if (frameInfo.needsYUVtoRGBconversion)
    {
        //
        // 1) Condense source data from 'nativeFrame' into 'conversionBuf.src'
        //

        for (size_t y = 0; y < imgHeight; y++)
            memcpy(conversionBuf.src.get() + y*frameInfo.srcBytesPerLine,
                   nativeFrame->image + y * nativeFrame->stride,
                   frameInfo.srcBytesPerLine);

        //
        // 2) Convert
        //

        dc1394_convert_to_RGB8(conversionBuf.src.get(), conversionBuf.dest.get(), imgWidth, imgHeight,
                               nativeFrame->yuv_byte_order, nativeFrame->color_coding, 8);

        //
        // 3) Prepare to copy converted data to 'frame'
        //

        srcLine = conversionBuf.dest.get();
        srcLineStep = imgWidth * 3;
        numDestCopyBytes = std::min(destLineStep, imgWidth * 3);
    }
    else
    {
        srcLine = nativeFrame->image;
        srcLineStep = nativeFrame->stride;
        numDestCopyBytes = std::min(destLineStep, (size_t)nativeFrame->stride);
    }

    for (int y = 0; y < frame->mat().rows; y++)
    {
        memcpy(destLine, srcLine, numDestCopyBytes);

        srcLine += srcLineStep;
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
    restart([this] { return std::make_shared<IIDCImagerWorker>(d->camera.get(), d->currentVidMode); });
    qDebug() << "Video streaming started successfully";
}
