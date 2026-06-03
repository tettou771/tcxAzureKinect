#pragma once

#include <TrussC.h>
#include <tcxDepthCamera.h>
#include <tcxAzureKinect.h>
using namespace std;
using namespace tc;
using namespace tcx;

// Point-cloud viewer driven by an Azure Kinect DK through the tcxDepthCamera
// interface: update() -> isFrameNew() -> toMesh(). Requires the hardware + the
// Azure Kinect Sensor SDK. If no device is found, the app still runs and shows
// a message rather than crashing.
class tcApp : public App {
public:
    void setup() override;
    void update() override;
    void draw() override;
    void keyPressed(int key) override;

private:
    void rebuild();
    void updateThumbnails();   // refresh the 2D color / depth / IR previews

    shared_ptr<AzureKinect> camera;
    bool deviceOk = false;
    EasyCam view;
    Mesh cloud;
    bool colored = true;    // Azure Kinect has color; show it by default
    int step = 2;           // decimate a bit (depth is up to 640x576)

    // 2D previews of the raw streams, drawn lined up in the corner.
    Image colorImg, depthImg, irImg;
};
