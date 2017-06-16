#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxDVS.hpp"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void setupGui();
		void update();
		void draw();
		void drawGui(ofEventArgs & args);

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		void updateFromGui();

		// GUI
		ofParameterGroup controls;
		ofParameter<float> targetSpeedGui;
		ofParameter<bool> APS;
		ofParameter<bool> DVS;
		ofParameter<bool> IMU;
		ofParameter<int> dvscolor;
		ofParameter<ofColor> color;
		ofParameter<bool> clear;
		ofParameter<bool> pause;
		ofParameter<bool> recording;
		ofParameter<bool> loadRec;
		ofParameter<bool> livemode;
		ofParameter<int> baFilterDt;
		ofParameter<int> dvsIntegration;
		ofxPanel gui;

	    // Silicon Retina
	    ofxDVS dvs;
};
