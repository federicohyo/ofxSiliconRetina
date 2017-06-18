#pragma once

#include "ofMain.h"
#include "ofxDVS.hpp"
#include "ofxDatGui.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

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

	    // Silicon Retina
	    ofxDVS dvs;

	    //Gui
        string getHex(int hex);
        void changeDrawGui();
        ofxDatGuiFolder* f1;
        void onButtonEvent(ofxDatGuiButtonEvent e);
        void onToggleEvent(ofxDatGuiToggleEvent e);
        void onSliderEvent(ofxDatGuiSliderEvent e);
        void onMatrixEvent(ofxDatGuiMatrixEvent e);
        void on2dPadEvent(ofxDatGui2dPadEvent e);
        void onTextInputEvent(ofxDatGuiTextInputEvent e);
        void onColorPickerEvent(ofxDatGuiColorPickerEvent e);
        ofxDatGuiTextInput * myTextTimer;
        ofxDatGuiValuePlotter * myIMU;
        bool drawGui;
    
        //counters
        int numPaused;
        int numPausedRec;
};
