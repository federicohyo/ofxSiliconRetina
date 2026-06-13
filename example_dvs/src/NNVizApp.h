#pragma once

#include "ofMain.h"

class ofxDVS;

class NNVizApp : public ofBaseApp {
public:
    void setup() override;
    void draw() override;
    void keyPressed(int key) override;

    ofxDVS* dvs = nullptr;
};
