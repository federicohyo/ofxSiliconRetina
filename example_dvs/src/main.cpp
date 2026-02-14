#include "ofMain.h"
#include "ofApp.h"
#include "ControlApp.h"
#include "ofAppGLFWWindow.h"

int main() {
    // Viewer window (large) — DVS visualization
    ofGLFWWindowSettings viewerSettings;
    viewerSettings.setSize(1024, 768);
    viewerSettings.setPosition(glm::vec2(320, 0));
    viewerSettings.title = "DVS Viewer";
    auto viewerWindow = ofCreateWindow(viewerSettings);

    // Control window (narrow) — all GUI panels
    ofGLFWWindowSettings controlSettings;
    controlSettings.setSize(300, 900);
    controlSettings.setPosition(glm::vec2(0, 0));
    controlSettings.title = "DVS Controls";
    controlSettings.shareContextWith = viewerWindow;
    auto controlWindow = ofCreateWindow(controlSettings);

    auto viewerApp = make_shared<ofApp>();
    auto controlApp = make_shared<ControlApp>();
    controlApp->dvs = &viewerApp->dvs;

    ofRunApp(viewerWindow, viewerApp);    // setup() → dvs.setupCore()
    ofRunApp(controlWindow, controlApp);  // setup() → dvs->setupGUI()
    ofRunMainLoop();
}
