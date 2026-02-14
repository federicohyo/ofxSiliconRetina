#include "ControlApp.h"
#include "ofxDVS.hpp"

void ControlApp::setup() {
    ofBackground(40);
    dvs->setupGUI();
}

void ControlApp::update() {
    dvs->updateGUI();
}

void ControlApp::draw() {
    dvs->drawControls();
}

void ControlApp::keyPressed(int key) {
    dvs->keyPressed(key);
}
