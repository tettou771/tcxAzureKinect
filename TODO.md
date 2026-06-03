# tcxAzureKinect — Known issues & roadmap

Status: **builds on Windows; full runtime verification pending.** Compiles and
links with the Azure Kinect Sensor SDK v1.4.1 (MSVC x64). `example-basic`
launches against a connected device and the k4a depth capture path produced
frames at the SDK level, but a clean end-to-end run (rendered point cloud,
color, IR) has not been confirmed yet — the first attempt was during a device
firmware update, and only depth captures were observed (with `capturesync`
queue-drop warnings). The items below are the outstanding work and caveats.

## Functional limitations

1. **Hardcoded capture config.** `openDevice()` fixes NFOV unbinned depth, 720p
   BGRA color, and 30 fps. Expose these (depth mode, color resolution, fps,
   color format) through a config struct / setters before `setup()`.

2. **Stream enable flags are not honored.** The base `DepthCamera` lets callers
   choose streams with `enableDepth()` / `enableColor()` / `enableInfrared()`
   (all off by default), but this backend always starts depth **and** color in
   `openDevice()` and always reads color/IR in `captureInto()` regardless of
   those flags. It should:
   - configure the device from the enabled set (depth is required for the k4a
     transformation; color via `color_resolution`, set to `OFF` when disabled —
     note this invalidates the color calibration, so guard `colorIntrinsics_` /
     `depthToColor_`; IR ships with the depth capture),
   - fill only the enabled streams, and set freshness accordingly.

3. **Color swizzle on the CPU every frame.** BGRA→RGBA is done in a per-pixel
   loop in `captureInto()`. Fine functionally; consider a cheaper path if it
   shows up in profiling.

4. **Single timestamp source.** `dst.timestamp` uses the depth image device
   timestamp only; color/IR timestamps are ignored.

## Verification

5. **Full hardware verification pending.** So far: compiles + links, DLLs
   bundled, and the example reaches the device (SDK-level depth captures seen).
   Still to confirm with a healthy (non-updating) device: a correctly rendered
   point cloud, color registration, and the IR image — plus resolving the
   `capturesync` full-queue drops seen on the first run (likely just the
   consumer not pulling fast enough, or the firmware-update state).

## Platforms & devices

6. **Linux build path is untested.** The CMake wiring has a Linux branch
   (`find_package(k4a)` / plain header+lib search) but it has not been built or
   run on Linux yet. Only Windows + SDK v1.4.1 is verified.

7. **Multi-device untested.** The `deviceIndex` constructor arg exists but only
   index 0 has been exercised. No device-enumeration / serial-number helper yet
   (`k4a_device_get_installed_count`, `k4a_device_get_serialnum`).

## Out of scope (for now)

8. No IMU, multi-camera sync (master/subordinate), or body tracking (k4abt).
   Note these if anyone needs them — they are separate features, not bugs.

## Build / environment notes

- **Windows SDK has no CMake config.** The MSI installer does not ship
  `k4aConfig.cmake`, so `CMakeLists.txt` locates the headers + `k4a.lib` under
  `C:\Program Files\Azure Kinect SDK vX.Y.Z` automatically (override with
  `-DK4A_SDK_ROOT="…/sdk"`). The runtime DLLs (`k4a.dll`, `depthengine_*.dll`)
  are copied next to the app exe at build time via `tc_addon_bundle_file()`.

- **Building `example-basic` via `trusscli build` (Debug):** on toolchains where
  the TrussC **core** Debug build is broken on MSVC (a `stb_vorbis.c` macro leak
  corrupting `<vector>`/`<atomic>` debug-iterator asserts), build
  RelWithDebInfo/Release instead. This is an upstream/core + toolchain issue, not
  a tcxAzureKinect bug — the addon itself compiles cleanly in all configs.
