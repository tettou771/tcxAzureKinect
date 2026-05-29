#pragma once

// =============================================================================
// tcxAzureKinect.h - Azure Kinect DK backend for tcxDepthCamera
// =============================================================================
//
// Drives an Azure Kinect DK (ToF) through the unified tcxDepthCamera interface.
// Built on the Azure Kinect Sensor SDK (k4a), which is officially Windows/Linux
// only. Color is delivered already registered to the depth geometry, and the
// point cloud is produced by the SDK's own depth->point-cloud transformation
// (so getWorldCoordinateAt() is overridden to read it rather than re-deproject).
//
//   #include <tcxAzureKinect.h>
//   using namespace tcx;
//
//   shared_ptr<DepthCamera> cam = make_shared<AzureKinect>();
//   cam->setThreaded(true);
//   cam->setup();
//   ...
//   cam->update();
//   if (cam->isFrameNew()) cam->toMesh({.colors = true}).draw();
//
// k4a handles are hidden behind a pImpl so this header stays SDK-free.
//
// =============================================================================

#include <tcxDepthCamera.h>
#include <cstdint>
#include <memory>
#include <vector>

namespace tcx {

using namespace tc;

// One captured frame. Holds only plain data (no k4a types) so it can live in
// the header and be triple-buffered by ThreadedDepthCameraBase. IR and the
// depth-registered color share the depth resolution.
struct AzureKinectFrame {
    int depthW = 0;
    int depthH = 0;
    std::vector<uint16_t> depth;   // distance in mm, depthW*depthH
    std::vector<int16_t>  xyz;      // SDK point cloud, 3*depthW*depthH, mm
    Pixels color;                   // BGRA->RGBA, registered to depth (depthW x depthH)
    Pixels ir;                      // active brightness, F32 1-channel (use getDataF32)
};

class AzureKinect : public ThreadedDepthCameraBase<AzureKinectFrame>,
                    public IColorStream,
                    public IInfraredStream {
public:
    explicit AzureKinect(uint32_t deviceIndex = 0);
    ~AzureKinect() override;

    // -- DepthCamera ----------------------------------------------------------
    int getWidth()  const override { return front().depthW; }
    int getHeight() const override { return front().depthH; }
    float getDistanceAt(int x, int y) const override;
    DepthIntrinsics getDepthIntrinsics() const override { return depthIntrinsics_; }
    DepthSensorType getSensorType() const override { return DepthSensorType::ToF; }
    // Uses the k4a XYZ point-cloud image (accurate; accounts for lens distortion).
    Vec3 getWorldCoordinateAt(int x, int y) const override;

    // -- IColorStream (color is already registered to the depth geometry) -----
    bool isColorFrameNew() const override { return isStreamNew(Stream::Color); }
    int getColorWidth()  const override { return front().depthW; }
    int getColorHeight() const override { return front().depthH; }
    const Pixels& getColorPixels() const override { return front().color; }
    Vec2 getColorTexCoordAt(int dx, int dy) const override;

    // -- IInfraredStream ------------------------------------------------------
    bool isInfraredFrameNew() const override { return isStreamNew(Stream::Infrared); }
    int getInfraredWidth()  const override { return front().depthW; }
    int getInfraredHeight() const override { return front().depthH; }
    const Pixels& getInfraredPixels() const override { return front().ir; }

protected:
    bool openDevice() override;
    void closeDevice() override;
    StreamFreshness captureInto(AzureKinectFrame& dst) override;

private:
    struct Impl;                    // hides k4a handles
    std::unique_ptr<Impl> impl_;
    uint32_t deviceIndex_;
    DepthIntrinsics depthIntrinsics_{};
};

} // namespace tcx
