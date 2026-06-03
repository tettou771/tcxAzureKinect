#include "tcApp.h"

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
    if (camera->isFrameNew()) rebuild();
}

// Rebuild the point-cloud mesh from the latest frame, in display units.
void tcApp::rebuild() {
    cloud = camera->toMesh({.colors = colored, .step = step});
    // tcxDepthCamera world coords are in meters with +Y down, with the camera
    // (sensor) at the origin. Scale up for the viewer and flip Y up; leaving the
    // origin where it is keeps the camera at (0,0,0) — the orbit pivot.
    cloud.scale(100.0f, -100.0f, 100.0f);
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

    // 2D stream previews along the bottom: color, depth, IR thumbnails. Each is
    // the camera's cached preview Image (getXxxImage()) - uploaded once per new
    // frame, no per-app conversion code. Drawn one after another, advancing x.
    if (deviceOk) {
        const float h = 130.0f, pad = 10.0f;
        const float y = getWindowHeight() - h - pad;
        float x = pad;
        setColor(1.0f);

        if (camera->hasColor()) {
            Image& color = camera->getColorImage();
            if (color.isAllocated()) {
                float w = h * color.getWidth() / (float)color.getHeight();
                color.draw(x, y, w, h);
                drawBitmapString("color", x + 4, y - 6);
                x += w + pad;
            }
        }

        Image& depth = camera->getDepthImage({.nearM = 0.3f, .farM = 4.0f});
        if (depth.isAllocated()) {
            float w = h * depth.getWidth() / (float)depth.getHeight();
            depth.draw(x, y, w, h);
            drawBitmapString("depth", x + 4, y - 6);
            x += w + pad;
        }

        if (camera->hasInfrared()) {
            Image& ir = camera->getInfraredImage();
            if (ir.isAllocated()) {
                float w = h * ir.getWidth() / (float)ir.getHeight();
                ir.draw(x, y, w, h);
                drawBitmapString("ir", x + 4, y - 6);
                x += w + pad;
            }
        }
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
