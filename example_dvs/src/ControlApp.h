#pragma once

#include "ofMain.h"

class ofxDVS;

class ControlApp : public ofBaseApp {
public:
    void setup() override;
    void update() override;
    void draw() override;
    void keyPressed(int key) override;

    ofxDVS* dvs = nullptr;
};
