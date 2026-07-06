#pragma once

// =============================================================================
// tcxAzureKinect.h - Azure Kinect DK backend for tcxDepthCamera
// =============================================================================
//
// Drives an Azure Kinect DK (ToF) through the unified tcxDepthCamera interface,
// built on the Azure Kinect Sensor SDK (k4a) — officially Windows/Linux only.
//
// It fills the canonical DepthFrame: depth (mm), color registered into the depth
// geometry, IR (active brightness), and the SDK-computed point cloud in
// frame.world (so the base uses the SDK's accurate, distortion-aware transform).
//
//   #include <tcxAzureKinect.h>
//   using namespace tcx;
//   shared_ptr<DepthCamera> cam = make_shared<AzureKinect>();
//   cam->setThreaded(true);
//   cam->setup();
//   ...
//   cam->update();
//   if (cam->isFrameNew()) cam->toMesh({.colors = true}).draw();
//
// k4a handles live in a pImpl, but this header includes <k4a/k4a.h> so it can
// hand back the raw k4a handles (getNativeDevice / Calibration / Transformation)
// as an escape hatch for SDK features the unified interface doesn't surface.
//
// =============================================================================

#include <tcxDepthCamera.h>
#include <k4a/k4a.h>
#include <cstdint>
#include <memory>

namespace tcx::azurekinect {

class AzureKinect : public tcx::depthcamera::DepthCamera {
public:
    explicit AzureKinect(uint32_t deviceIndex = 0);
    ~AzureKinect() override;

    depthcamera::DepthSensorType getSensorType() const override { return depthcamera::DepthSensorType::ToF; }

    // -------------------------------------------------------------------------
    // Escape hatch: raw k4a handles for SDK features the unified DepthCamera
    // interface doesn't surface (IMU, body tracking via k4abt, exposure / white
    // balance control, native transforms, multi-device hardware sync, ...).
    // Valid only while the device is open: null / empty before setup() and
    // after closeDevice(). The backend owns their lifetime — do NOT close or
    // free them yourself.
    // -------------------------------------------------------------------------
    k4a_device_t             getNativeDevice() const;
    const k4a_calibration_t& getNativeCalibration() const;
    k4a_transformation_t     getNativeTransformation() const;

    // The k4a_capture behind the most recent successful frame, retained until the
    // next one is grabbed — e.g. to feed k4abt_tracker_enqueue_capture() for body
    // tracking, sharing the single capture instead of opening the device twice.
    // Null before the first frame / after closeDevice(); the backend owns it, do
    // NOT release it. In threaded mode (setThreaded(true)) it is swapped on the
    // grab thread, so read it on the thread that drives update() — or run
    // non-threaded when handing it to the body tracker.
    k4a_capture_t            getNativeCapture() const;

protected:
    bool openDevice() override;
    void closeDevice() override;
    depthcamera::StreamFreshness captureInto(depthcamera::DepthFrame& dst) override;

private:
    struct Impl;                    // hides k4a handles
    std::unique_ptr<Impl> impl_;
    uint32_t deviceIndex_;
    depthcamera::DepthIntrinsics depthIntrinsics_{};  // cached from calibration in openDevice
    depthcamera::DepthIntrinsics colorIntrinsics_{};  // color camera intrinsics (native res)
    tc::Mat4 depthToColor_{};             // depth-cam space -> color-cam space
};

} // namespace tcx::azurekinect

// -----------------------------------------------------------------------------
// Backward compatibility. The canonical namespace is now `tcx::azurekinect`.
// These silent aliases keep older code compiling: flat `tcx::AzureKinect` and
// legacy `trussc::AzureKinect`. DEPRECATED — removed in v1.0.0.
// (No [[deprecated]] attribute: under the usual `using namespace tc;` it would
//  warn on idiomatic unqualified use too. See tcxAzureKinect README for migration.)
// -----------------------------------------------------------------------------
namespace tcx    { using azurekinect::AzureKinect; } // deprecated: remove at v1.0.0
namespace trussc { using tcx::azurekinect::AzureKinect; } // deprecated: remove at v1.0.0
