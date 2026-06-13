#include "NNVizApp.h"
#include "ofxDVS.hpp"

void NNVizApp::setup() {
    ofBackground(18, 18, 22);
}

void NNVizApp::draw() {
    dvs->drawNNViz();
}

void NNVizApp::keyPressed(int key) {
    dvs->keyPressed(key);
}
