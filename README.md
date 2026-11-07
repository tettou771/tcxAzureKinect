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

## Using a Femto Bolt (Orbbec K4A Wrapper)

This addon talks to the **k4a API**, not to a specific device — so it also drives
an **Orbbec Femto Bolt** (the de-facto Azure Kinect DK successor) without any code
change. The Femto is *not* a genuine Azure Kinect, so the Microsoft k4a SDK will
**not** enumerate it; instead install Orbbec's **K4A Wrapper**, a drop-in fork of
the Azure Kinect Sensor SDK that exposes the same `k4a.h` / `k4a.lib` / `k4a.dll`
but binds to Orbbec cameras.

> **Do not install the Microsoft k4a SDK as well** — if its `k4a.dll` is picked up
> instead of the wrapper's, the Femto won't open. Use only the wrapper.

1. **Download** the wrapper (use the **v2** branch for Femto Bolt) from
   [OrbbecSDK-K4A-Wrapper releases](https://github.com/orbbec/OrbbecSDK-K4A-Wrapper/releases)
   — the `..._windows_*.zip` (amd64/x64) asset — and extract it somewhere stable,
   e.g. `C:\SDK\OrbbecK4A`. Supported: Windows 10+, Linux x64, Linux ARM64 (Jetson).
   Requires **Femto Bolt firmware ≥ 1.1.2** (recommended 1.1.3).
2. **(Windows) Register UVC metadata** — run `scripts\obsensor_metadata_win10.ps1`
   once from an **admin** PowerShell. Without it the color stream may not appear.
3. **Verify the hardware first**, independent of TrussC: run the bundled
   `bin\k4aviewer.exe`. If you see live depth / color, the camera + wrapper +
   driver are all good. (Orbbec's own guide: `doc\Access_AKDK_Application_Software_with_Femto_Bolt.pdf`.)
4. **Point the build at the wrapper.** Unlike the Microsoft MSI, the wrapper zip
   *does* ship a CMake config (`lib\cmake\k4a\k4aConfig.cmake`), so this addon's
   `find_package(k4a CONFIG)` finds it directly — just expose its location via
   `CMAKE_PREFIX_PATH`. Point it at the wrapper **root** (the folder that contains
   `bin\`, `include\`, `lib\`), not the `lib\cmake\k4a` subfolder.

   ```powershell
   # Current shell only (one-off):
   $env:CMAKE_PREFIX_PATH = "C:\SDK\OrbbecK4A"
   ```

   To persist it for all future shells, set the **User** environment variable.
   `setx` works but *overwrites* (and truncates at 1024 chars), so if you already
   use `CMAKE_PREFIX_PATH` for something else, append safely instead:

   ```powershell
   # Append to the existing User value without clobbering it (no-op if present):
   $add = "C:\SDK\OrbbecK4A"
   $old = [Environment]::GetEnvironmentVariable("CMAKE_PREFIX_PATH", "User")
   if (-not $old)                            { $new = $add }
   elseif (($old -split ';') -contains $add) { $new = $old }
   else                                      { $new = "$old;$add" }
   [Environment]::SetEnvironmentVariable("CMAKE_PREFIX_PATH", $new, "User")
   ```

   This persists like `setx` but only touches the User scope and won't duplicate.
   Either way it applies to **new** shells only — reopen your terminal (or set
   `$env:CMAKE_PREFIX_PATH` in the current one too).

   > If the build was already configured once before this was set, clear the
   > build dir so CMake reconfigures from scratch: `trusscli clean`.

5. **Copy the runtime DLLs next to the app executable.** The automatic DLL
   bundling only runs on the manual Microsoft-MSI path; when k4a is found via
   `find_package(k4a CONFIG)` (the wrapper case) it is skipped, so copy the
   wrapper's **entire `bin\`** — the DLLs (`k4a.dll`, `depthengine_2_0.dll`,
   `OrbbecSDK.dll`, `k4arecord.dll`) **and the `extensions\` folder** — next to the
   built `example-basic` executable. Missing `extensions\` / `OrbbecSDK.dll` makes
   the device fail to start at runtime.

Everything else below (API, units, point cloud, etc.) is identical to a real
Azure Kinect.

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
