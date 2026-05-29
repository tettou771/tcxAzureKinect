// =============================================================================
// tcxAzureKinect.cpp - Azure Kinect DK backend implementation (k4a)
// =============================================================================
//
// NOTE: First pass written against the k4a API; NOT yet compiled or run against
// the SDK / hardware (developed on macOS, which k4a does not support). Build /
// refine on Linux with the Azure Kinect Sensor SDK installed. The tcxDepthCamera
// plumbing it fills is the validated part.
//
// =============================================================================

#include "tcxAzureKinect.h"

#include <k4a/k4a.h>

using namespace std;

namespace tcx {

// Timeout for a synchronized capture before the worker re-checks running (ms).
static constexpr int32_t kCaptureTimeoutMs = 1000;

struct AzureKinect::Impl {
    k4a_device_t               device         = nullptr;
    k4a_transformation_t       transformation = nullptr;
    k4a_calibration_t          calibration{};
    k4a_device_configuration_t config{};
};

AzureKinect::AzureKinect(uint32_t deviceIndex)
    : impl_(make_unique<Impl>()), deviceIndex_(deviceIndex) {}

AzureKinect::~AzureKinect() = default;

// -----------------------------------------------------------------------------
bool AzureKinect::openDevice() {
    if (k4a_device_open(deviceIndex_, &impl_->device) != K4A_RESULT_SUCCEEDED) {
        logError("tcxAzureKinect") << "failed to open device " << deviceIndex_;
        return false;
    }

    impl_->config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    impl_->config.color_format             = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    impl_->config.color_resolution         = K4A_COLOR_RESOLUTION_720P;
    impl_->config.depth_mode               = K4A_DEPTH_MODE_NFOV_UNBINNED;
    impl_->config.camera_fps               = K4A_FRAMES_PER_SECOND_30;
    impl_->config.synchronized_images_only = true;

    if (k4a_device_start_cameras(impl_->device, &impl_->config) != K4A_RESULT_SUCCEEDED) {
        logError("tcxAzureKinect") << "failed to start cameras";
        k4a_device_close(impl_->device);
        impl_->device = nullptr;
        return false;
    }

    if (k4a_device_get_calibration(impl_->device, impl_->config.depth_mode,
                                   impl_->config.color_resolution,
                                   &impl_->calibration) != K4A_RESULT_SUCCEEDED) {
        logError("tcxAzureKinect") << "failed to get calibration";
        closeDevice();
        return false;
    }

    impl_->transformation = k4a_transformation_create(&impl_->calibration);

    const k4a_calibration_camera_t& dc = impl_->calibration.depth_camera_calibration;
    const auto& p = dc.intrinsics.parameters.param;
    depthIntrinsics_.width  = dc.resolution_width;
    depthIntrinsics_.height = dc.resolution_height;
    depthIntrinsics_.fx = p.fx; depthIntrinsics_.fy = p.fy;
    depthIntrinsics_.cx = p.cx; depthIntrinsics_.cy = p.cy;
    depthIntrinsics_.k1 = p.k1; depthIntrinsics_.k2 = p.k2; depthIntrinsics_.k3 = p.k3;
    depthIntrinsics_.p1 = p.p1; depthIntrinsics_.p2 = p.p2;

    return true;
}

void AzureKinect::closeDevice() {
    if (impl_->transformation) {
        k4a_transformation_destroy(impl_->transformation);
        impl_->transformation = nullptr;
    }
    if (impl_->device) {
        k4a_device_stop_cameras(impl_->device);
        k4a_device_close(impl_->device);
        impl_->device = nullptr;
    }
}

// -----------------------------------------------------------------------------
StreamFreshness AzureKinect::captureInto(DepthFrame& dst) {
    StreamFreshness fresh;

    k4a_capture_t cap = nullptr;
    if (k4a_device_get_capture(impl_->device, &cap, kCaptureTimeoutMs)
            != K4A_WAIT_RESULT_SUCCEEDED) {
        if (cap) k4a_capture_release(cap);
        return fresh;  // timeout / no frame
    }

    k4a_image_t depthImg = k4a_capture_get_depth_image(cap);
    if (depthImg) {
        const int w = k4a_image_get_width_pixels(depthImg);
        const int h = k4a_image_get_height_pixels(depthImg);
        dst.w = w;
        dst.h = h;
        dst.depthScale = 0.001f;  // k4a depth is uint16 mm
        dst.intrinsics = depthIntrinsics_;
        dst.timestamp =
            k4a_image_get_device_timestamp_usec(depthImg) * 1e-6;

        const uint16_t* db =
            reinterpret_cast<const uint16_t*>(k4a_image_get_buffer(depthImg));
        dst.depth.assign(db, db + static_cast<size_t>(w) * h);
        fresh.depth = true;

        // SDK point cloud (XYZ int16 mm) -> frame.world (meters). The base will
        // use this instead of re-deprojecting (accurate, distortion-aware).
        k4a_image_t xyzImg = nullptr;
        if (k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, w, h,
                             w * 3 * static_cast<int>(sizeof(int16_t)),
                             &xyzImg) == K4A_RESULT_SUCCEEDED) {
            if (k4a_transformation_depth_image_to_point_cloud(
                    impl_->transformation, depthImg,
                    K4A_CALIBRATION_TYPE_DEPTH, xyzImg) == K4A_RESULT_SUCCEEDED) {
                const int16_t* xb =
                    reinterpret_cast<const int16_t*>(k4a_image_get_buffer(xyzImg));
                const size_t n = static_cast<size_t>(w) * h;
                dst.world.resize(n);
                for (size_t i = 0; i < n; ++i) {
                    dst.world[i] = Vec3{xb[i * 3 + 0] * 0.001f,
                                        xb[i * 3 + 1] * 0.001f,
                                        xb[i * 3 + 2] * 0.001f};
                }
            }
            k4a_image_release(xyzImg);
        }

        // Color registered into the depth geometry.
        k4a_image_t colorImg = k4a_capture_get_color_image(cap);
        if (colorImg) {
            k4a_image_t colorInDepth = nullptr;
            if (k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32, w, h, w * 4,
                                 &colorInDepth) == K4A_RESULT_SUCCEEDED) {
                if (k4a_transformation_color_image_to_depth_camera(
                        impl_->transformation, depthImg, colorImg,
                        colorInDepth) == K4A_RESULT_SUCCEEDED) {
                    dst.color.allocate(w, h, 4);  // RGBA
                    const uint8_t* cb = k4a_image_get_buffer(colorInDepth);
                    uint8_t* out = dst.color.getData();
                    const size_t n = static_cast<size_t>(w) * h;
                    for (size_t i = 0; i < n; ++i) {
                        out[i * 4 + 0] = cb[i * 4 + 2];  // R <- B
                        out[i * 4 + 1] = cb[i * 4 + 1];  // G
                        out[i * 4 + 2] = cb[i * 4 + 0];  // B <- R
                        out[i * 4 + 3] = cb[i * 4 + 3];  // A
                    }
                    fresh.color = true;
                }
                k4a_image_release(colorInDepth);
            }
            k4a_image_release(colorImg);
        }

        k4a_image_release(depthImg);
    }

    // IR / active brightness (F32 1-channel, use getDataF32()).
    k4a_image_t irImg = k4a_capture_get_ir_image(cap);
    if (irImg) {
        const int w = k4a_image_get_width_pixels(irImg);
        const int h = k4a_image_get_height_pixels(irImg);
        dst.ir.allocate(w, h, 1, PixelFormat::F32);
        const uint16_t* ib =
            reinterpret_cast<const uint16_t*>(k4a_image_get_buffer(irImg));
        float* irOut = dst.ir.getDataF32();
        const size_t n = static_cast<size_t>(w) * h;
        for (size_t i = 0; i < n; ++i) irOut[i] = static_cast<float>(ib[i]);
        fresh.infrared = true;
        k4a_image_release(irImg);
    }

    k4a_capture_release(cap);
    return fresh;
}

} // namespace tcx
