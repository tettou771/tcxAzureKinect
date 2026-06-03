# tcxAzureKinect

Azure Kinect DK support for [TrussC](https://github.com/TrussC-org/TrussC),
implementing the [`tcxDepthCamera`](https://github.com/TrussC-org/TrussC/tree/dev/addons/tcxDepthCamera)
interface. Drive an Azure Kinect (a time-of-flight depth camera) through the
same `DepthCamera` API as any other depth sensor.

> **Status: working on Windows (hardware-verified).** Builds and links with the
> Azure Kinect Sensor SDK v1.4.1 (MSVC x64); `example-basic` runs against a
> connected device and renders a live colored point cloud — depth, color, and IR
> are all confirmed end-to-end. The example also shows color / depth / IR
> previews and orbits around the camera origin. Linux build wiring is present but
> not yet exercised. k4a has no macOS support. See `example-basic/`.

## Requirements

- **Azure Kinect Sensor SDK (k4a)** installed. Officially supported on **Linux
  and Windows only** — not macOS.
  - **Windows:** install the SDK MSI from
    [Microsoft](https://learn.microsoft.com/azure/kinect-dk/sensor-sdk-download).
    It installs to `C:\Program Files\Azure Kinect SDK vX.Y.Z` but does **not**
    ship a CMake config, so this addon's `CMakeLists.txt` locates the headers +
    `k4a.lib` under that path automatically (override with
    `-DK4A_SDK_ROOT="…/sdk"` if installed elsewhere). The runtime DLLs
    (`k4a.dll` + `depthengine_*.dll`) are copied next to the app executable at
    build time — no manual PATH setup needed.
  - **Linux:** install `libk4a<ver>-dev` (provides a `k4a` CMake package /
    `libk4a`), which is found via `find_package(k4a)`.
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
- **Color** is kept at its NATIVE full resolution (e.g. 720p) - it is NOT
  registered/downsampled to the depth geometry. The base computes the depth->
  color mapping on demand from the color intrinsics + depth->color extrinsic
  (both cached from the k4a calibration), so `getColorPixels()` returns the full
  color frame and `getColorTexCoordAt()` / `getColorAt()` project correctly.
  Use `registerColorToDepth()` if you specifically want a depth-aligned image.
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
