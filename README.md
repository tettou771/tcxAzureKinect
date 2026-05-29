# tcxAzureKinect

Azure Kinect DK support for [TrussC](https://github.com/TrussC-org/TrussC),
implementing the [`tcxDepthCamera`](https://github.com/TrussC-org/TrussC/tree/dev/addons/tcxDepthCamera)
interface. Drive an Azure Kinect (a time-of-flight depth camera) through the
same `DepthCamera` API as any other depth sensor.

> 🚧 **Status: first pass, not yet hardware-tested.** The k4a wiring was written
> against the SDK API but has not been compiled/run against a device yet (k4a
> has no macOS support; first build target is Linux). Expect refinement.

## Requirements

- **Azure Kinect Sensor SDK (k4a)** installed (provides the `k4a::k4a` CMake
  target). Officially supported on **Linux and Windows only** — not macOS.
- The `tcxDepthCamera` addon (this addon depends on it).

## Usage

```cpp
#include <tcxAzureKinect.h>
using namespace tcx;

shared_ptr<DepthCamera> cam = make_shared<AzureKinect>();  // device index 0
cam->setThreaded(true);     // grab on a background thread (call before setup)
cam->setup();
// ...
cam->update();
if (cam->isFrameNew()) {
    Mesh cloud = cam->toMesh({.colors = true});
    cloud.draw();
}
```

Color and IR are part of the canonical `DepthFrame`, so they are read directly
on the camera (no capability cast needed):

```cpp
if (cam->hasColor())    { const Pixels& c  = cam->getColorPixels(); }
if (cam->hasInfrared()) { const Pixels& ir = cam->getInfraredPixels(); }
const DepthFrame& f = cam->currentFrame();   // depth / world / color / ir / intrinsics
```

## Notes

- This backend just **fills the canonical `DepthFrame`** in `captureInto()`; the
  `tcxDepthCamera` base provides all the accessors, meshing and threading.
- **Units:** depth distance and world coordinates are in **meters** (the
  tcxDepthCamera convention). Depth is stored as uint16 mm with
  `depthScale = 0.001`.
- **Color** is delivered already registered into the depth geometry (via
  `k4a_transformation_color_image_to_depth_camera`), so per-vertex color and UVs
  map straight through the depth pixel coordinates. `getColorPixels()` therefore
  returns a depth-resolution image, not the full 720p color frame.
- **Point cloud** uses the SDK's `depth_image_to_point_cloud` transformation
  (accurate, accounts for lens distortion): the result is written to
  `frame.world`, and the base returns it from `getWorldCoordinateAt()` /
  `toMesh()` instead of re-deprojecting from intrinsics.
- **IR** is the active-brightness image, stored as a 1-channel F32 `Pixels`
  (read via `getDataF32()`).
- Sensor type reports `DepthSensorType::ToF`.

## Configuration

The current first pass hardcodes NFOV unbinned depth, 720p BGRA color, 30 fps.
Making these configurable is a planned follow-up.

## License

MIT. See `LICENSES.md`. Depends on the k4a SDK (MIT, Microsoft) which must be
installed separately.
