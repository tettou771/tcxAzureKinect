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

    view.setDistance(400.0f);
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
    // tcxDepthCamera world coords are in meters with +Y down. Scale up for the
    // viewer and flip Y up; pull the scene back a bit so it frames nicely.
    cloud.scale(100.0f, -100.0f, 100.0f);
    cloud.translate(0.0f, 0.0f, -200.0f);
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
