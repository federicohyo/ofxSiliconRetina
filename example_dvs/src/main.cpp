#include "ofMain.h"
#include "ofApp.h"
#include "ControlApp.h"
#include "NNVizApp.h"
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

    // NN Viz window — VTEI inputs, analog probe activations, SNN spike maps
    // 3 groups of 4 probes each; sized to fit all strips on a 1080p display.
    ofGLFWWindowSettings nnvizSettings;
    nnvizSettings.setSize(600, 840);
    nnvizSettings.setPosition(glm::vec2(1350, 0));
    nnvizSettings.title = "NN Visualization";
    nnvizSettings.shareContextWith = viewerWindow;
    auto nnvizWindow = ofCreateWindow(nnvizSettings);

    auto viewerApp  = make_shared<ofApp>();
    auto controlApp = make_shared<ControlApp>();
    auto nnvizApp   = make_shared<NNVizApp>();
    controlApp->dvs = &viewerApp->dvs;
    nnvizApp->dvs   = &viewerApp->dvs;

    ofRunApp(viewerWindow,  viewerApp);   // setup() → dvs.setupCore()
    ofRunApp(controlWindow, controlApp);  // setup() → dvs->setupGUI()
    ofRunApp(nnvizWindow,   nnvizApp);    // draw()  → dvs->drawNNViz()
    ofRunMainLoop();
}
