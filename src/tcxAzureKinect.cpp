// =============================================================================
// tcxAzureKinect.cpp - Azure Kinect DK backend implementation (k4a)
// =============================================================================
//
// Compiles and links against the Azure Kinect Sensor SDK v1.4.1 (MSVC x64) on
// Windows; full hardware verification (point cloud / color / IR end-to-end) is
// still pending. k4a does not support macOS. The config (NFOV unbinned depth,
// 720p BGRA color, 30 fps) is still hardcoded; making it configurable is a
// planned follow-up.
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

// Escape hatch: raw k4a handles (see header). Valid only while the device is
// open; null / empty otherwise. The backend keeps ownership.
k4a_device_t AzureKinect::getNativeDevice() const { return impl_->device; }
const k4a_calibration_t& AzureKinect::getNativeCalibration() const { return impl_->calibration; }
k4a_transformation_t AzureKinect::getNativeTransformation() const { return impl_->transformation; }

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

    // Color camera intrinsics (native color resolution).
    const k4a_calibration_camera_t& cc = impl_->calibration.color_camera_calibration;
    const auto& cp = cc.intrinsics.parameters.param;
    colorIntrinsics_.width  = cc.resolution_width;
    colorIntrinsics_.height = cc.resolution_height;
    colorIntrinsics_.fx = cp.fx; colorIntrinsics_.fy = cp.fy;
    colorIntrinsics_.cx = cp.cx; colorIntrinsics_.cy = cp.cy;
    colorIntrinsics_.k1 = cp.k1; colorIntrinsics_.k2 = cp.k2; colorIntrinsics_.k3 = cp.k3;
    colorIntrinsics_.p1 = cp.p1; colorIntrinsics_.p2 = cp.p2;

    // depth-cam space -> color-cam space (k4a translation is in mm -> meters).
    const k4a_calibration_extrinsics_t& ex =
        impl_->calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR];
    Mat4 dc2c;  // row-major affine (identity by default)
    dc2c.m[0]  = ex.rotation[0]; dc2c.m[1]  = ex.rotation[1]; dc2c.m[2]  = ex.rotation[2];
    dc2c.m[4]  = ex.rotation[3]; dc2c.m[5]  = ex.rotation[4]; dc2c.m[6]  = ex.rotation[5];
    dc2c.m[8]  = ex.rotation[6]; dc2c.m[9]  = ex.rotation[7]; dc2c.m[10] = ex.rotation[8];
    dc2c.m[3]  = ex.translation[0] * 0.001f;
    dc2c.m[7]  = ex.translation[1] * 0.001f;
    dc2c.m[11] = ex.translation[2] * 0.001f;
    depthToColor_ = dc2c;

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

        // Native full-resolution color (NOT registered to depth). The depth->
        // color mapping is computed on demand by the base from colorIntrinsics +
        // depthToColor, so we keep the color at its own resolution.
        k4a_image_t colorImg = k4a_capture_get_color_image(cap);
        if (colorImg) {
            const int cw = k4a_image_get_width_pixels(colorImg);
            const int chh = k4a_image_get_height_pixels(colorImg);
            dst.color.allocate(cw, chh, 4);  // RGBA
            const uint8_t* cb = k4a_image_get_buffer(colorImg);
            uint8_t* out = dst.color.getData();
            const size_t n = static_cast<size_t>(cw) * chh;
            for (size_t i = 0; i < n; ++i) {
                out[i * 4 + 0] = cb[i * 4 + 2];  // R <- B
                out[i * 4 + 1] = cb[i * 4 + 1];  // G
                out[i * 4 + 2] = cb[i * 4 + 0];  // B <- R
                out[i * 4 + 3] = cb[i * 4 + 3];  // A
            }
            dst.colorIntrinsics = colorIntrinsics_;
            dst.depthToColor = depthToColor_;
            fresh.color = true;
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
