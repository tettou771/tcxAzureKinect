#include "tcApp.h"
#include <cmath>
#include <cstring>

// -----------------------------------------------------------------------------
// Stream -> Image helpers for the 2D previews. Each (re)allocates the Image only
// when the source size changes, fills the pixel buffer, and uploads it.
// -----------------------------------------------------------------------------
namespace {

void ensureSize(Image& img, int w, int h) {
    if (!img.isAllocated() || img.getWidth() != w || img.getHeight() != h) {
        img.allocate(w, h, 4);
    }
}

// Color is already native RGBA (8-bit); copy it straight across.
void uploadColor(Image& img, const Pixels& c) {
    if (!c.isAllocated() || c.getChannels() != 4) return;
    ensureSize(img, c.getWidth(), c.getHeight());
    std::memcpy(img.getPixelsData(), c.getData(),
                static_cast<size_t>(c.getWidth()) * c.getHeight() * 4);
    img.setDirty();
    img.update();
}

// Depth is uint16 (depthScale meters/unit); map a near..far band to grayscale
// (near = bright). Invalid samples (0) stay black.
void uploadDepth(Image& img, const tcx::DepthFrame& f, float nearM, float farM) {
    if (f.w <= 0 || f.h <= 0 || f.depth.empty()) return;
    ensureSize(img, f.w, f.h);
    unsigned char* d = img.getPixelsData();
    const float span = (farM > nearM) ? (farM - nearM) : 1.0f;
    const int n = f.w * f.h;
    for (int i = 0; i < n; ++i) {
        unsigned char g = 0;
        if (f.depth[i] != 0) {
            float t = (f.depth[i] * f.depthScale - nearM) / span;
            t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
            g = static_cast<unsigned char>((1.0f - t) * 255.0f);
        }
        d[i * 4 + 0] = g; d[i * 4 + 1] = g; d[i * 4 + 2] = g; d[i * 4 + 3] = 255;
    }
    img.setDirty();
    img.update();
}

// IR is single-channel F32 active brightness; auto-normalize by the frame max
// and gamma-soften so low returns stay visible.
void uploadIR(Image& img, const Pixels& ir) {
    if (!ir.isAllocated()) return;
    const int w = ir.getWidth(), h = ir.getHeight(), n = w * h;
    ensureSize(img, w, h);
    const float* s = ir.getDataF32();
    float mx = 1.0f;
    for (int i = 0; i < n; ++i) if (s[i] > mx) mx = s[i];
    const float inv = 1.0f / mx;
    unsigned char* d = img.getPixelsData();
    for (int i = 0; i < n; ++i) {
        float t = s[i] * inv;
        t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
        unsigned char g = static_cast<unsigned char>(std::sqrt(t) * 255.0f);
        d[i * 4 + 0] = g; d[i * 4 + 1] = g; d[i * 4 + 2] = g; d[i * 4 + 3] = 255;
    }
    img.setDirty();
    img.update();
}

} // namespace

void tcApp::setup() {
    setWindowTitle("tcxAzureKinect - Azure Kinect DK point cloud");

    camera = make_shared<AzureKinect>();   // device index 0
    camera->setThreaded(true);             // grab on a background thread (before setup)
    camera->enableDepth();                 // streams are all off by default; enable what we use
    camera->enableColor();
    camera->enableInfrared();
    deviceOk = camera->setup();            // false if no device / SDK runtime missing
    if (!deviceOk) {
        logError("example-basic") << "Azure Kinect not available — is a device "
                                     "connected and the SDK runtime installed?";
    }

    // Orbit around the camera origin (0,0,0) — where the sensor sits in the
    // point cloud. distance frames the scene.
    view.setTarget(0.0f, 0.0f, 0.0f);
    view.setDistance(500.0f);
    view.enableMouseInput();
}

void tcApp::update() {
    if (!deviceOk) return;
    camera->update();
    if (camera->isFrameNew()) {
        rebuild();
        updateThumbnails();
    }
}

// Rebuild the point-cloud mesh from the latest frame, in display units.
void tcApp::rebuild() {
    cloud = camera->toMesh({.colors = colored, .step = step});
    // tcxDepthCamera world coords are in meters with +Y down, with the camera
    // (sensor) at the origin. Scale up for the viewer and flip Y up; leaving the
    // origin where it is keeps the camera at (0,0,0) — the orbit pivot.
    cloud.scale(100.0f, -100.0f, 100.0f);
}

// Refresh the 2D previews from the latest frame (called when a new frame lands).
void tcApp::updateThumbnails() {
    if (camera->hasColor())    uploadColor(colorImg, camera->getColorPixels());
    uploadDepth(depthImg, camera->currentFrame(), 0.3f, 4.0f);
    if (camera->hasInfrared()) uploadIR(irImg, camera->getInfraredPixels());
}

void tcApp::draw() {
    clear(0.08f, 0.09f, 0.11f);

    if (deviceOk) {
        view.begin();
        pushMatrix();
        if (!colored) setColor(0.55f, 0.75f, 1.0f);
        cloud.draw();
        popMatrix();
        view.end();
    }

    // 2D stream previews: a row of equal-height thumbnails along the bottom.
    if (deviceOk) {
        const float h = 130.0f, pad = 10.0f;
        float x = pad;
        const float y = getWindowHeight() - h - pad;
        auto thumb = [&](Image& img, const char* label) {
            if (!img.isAllocated()) return;
            const float w = h * img.getWidth() / static_cast<float>(img.getHeight());
            setColor(1.0f);
            img.draw(x, y, w, h);
            drawBitmapString(label, x + 4, y - 6);   // label just above the image
            x += w + pad;
        };
        thumb(colorImg, "color");
        thumb(depthImg, "depth");
        thumb(irImg,    "ir");
    }

    setColor(1.0f);
    string hud;
    if (deviceOk) {
        hud  = "Azure Kinect DK\n";
        hud += to_string(cloud.getNumVertices()) + " points";
        hud += colored ? "  [colored]" : "  [plain]";
        hud += "\nC: toggle color    1-4: decimation    drag: orbit";
    } else {
        hud  = "No Azure Kinect device found.\n";
        hud += "Connect the camera + power, install the Azure Kinect SDK\n";
        hud += "runtime, and relaunch.";
    }
    drawBitmapString(hud, 20, 20);
}

void tcApp::keyPressed(int key) {
    if (key == 'c' || key == 'C') {
        colored = !colored;
    } else if (key >= '1' && key <= '4') {
        step = key - '0';
    }
    if (deviceOk) rebuild();
}
