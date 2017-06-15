#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    dvs.setup();

    //GUI
    int x = 0;
    int y = 0;
    ofSetWindowPosition(0, 0);

    vector<string> options = {"ONE", "TWO", "THREE", "FOUR"};


    f1 = new ofxDatGuiFolder("Control", ofColor::fromHex(0xFFD00B));
    f1->addBreak();
    f1->addFRM();
    f1->addBreak();
    f1->addSlider("1/speed", 0, 3, 0.2);
    f1->addToggle("APS", true);
    f1->addBreak();
    f1->addToggle("DVS", true);
    f1->addBreak();
    f1->addToggle("IMU", true);
    f1->addBreak();
    f1->addMatrix("DVS Color", 7, true);
    f1->addBreak();
    f1->addButton("Clear");
	f1->addBreak();
    f1->addButton("Pause");
	f1->addBreak();
    f1->addButton("Start Recording");
	f1->addBreak();
    f1->addButton("Load Recording");
	f1->addBreak();
    f1->addButton("Live");
	f1->addBreak();
    f1->setPosition(x, y);
    f1->expand();

    f1->onButtonEvent(this, &ofApp::onButtonEvent);
    f1->onToggleEvent(this, &ofApp::onToggleEvent);
    f1->onSliderEvent(this, &ofApp::onSliderEvent);
    f1->onMatrixEvent(this, &ofApp::onMatrixEvent);

    numPaused = 0;
}

//--------------------------------------------------------------
void ofApp::update(){
    dvs.update();
    dvs.updateBAFilter();
    //dvs.updateImageGenerator();

    //GUI
    f1->update();
}

//--------------------------------------------------------------
void ofApp::draw(){
    dvs.drawFrames();
    //dvs.drawImageGenerator();
    dvs.drawSpikes();

    //GUI
    f1->draw();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    /*if (key == 'r') {
        dvs.changeRecordingStatus(); //enable/disable recording
    }*/
   /* if (key == 'n'){ //load file from disk
        dvs.loadFile();
    }
    if (key == 'l'){ //connect to device
        dvs.tryLive();
    }*/
    if (key == '-') {
    	float tgc = dvs.getTargetSpeed();
        float change = tgc*0.1;
    	if(tgc+change < LONG_MAX){
    		dvs.changeTargetSpeed(+change); //faster
        }
    }
    if (key == '+') {
    	float tgc = dvs.getTargetSpeed();
        float change = tgc*0.1;
    	if(tgc-change > 0){
    		dvs.changeTargetSpeed(-change); //slower
        }
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

void ofApp::onButtonEvent(ofxDatGuiButtonEvent e)
{
	if(e.target->getLabel() == "Clear"){
		dvs.clearDraw();
	}else if( (e.target->getLabel() == "Pause") ||  (e.target->getLabel() == "Start")){
		numPaused++;
		if((numPaused % 2) == 0){
			e.target->setLabel("Pause");
		}else{
			e.target->setLabel("Start");
		}
		dvs.changePause();
	}else if( (e.target->getLabel() == "Start Recording") ||  (e.target->getLabel() == "Stop Recording")){
		numPaused++;
		if((numPaused % 2) == 0){
			e.target->setLabel("Start Recording");
		}else{
			e.target->setLabel("Stop Recording");
		}
		dvs.changeRecordingStatus();
	}else if(e.target->getLabel() == "Load Recording"){
		dvs.loadFile();
	}else if(e.target->getLabel() == "Live"){
		dvs.tryLive();
	}
}

void ofApp::onToggleEvent(ofxDatGuiToggleEvent e)
{
    //cout << "onToggleEvent " << e.target->getLabel() << e.checked << endl;
    if(e.target->getLabel() == "APS"){
    	dvs.changeAps();
    }else if(e.target->getLabel() == "DVS"){
    	dvs.changeDvs();
    }else if(e.target->getLabel() == "IMU"){
    	dvs.changeImu();
    }

}

void ofApp::onSliderEvent(ofxDatGuiSliderEvent e)
{
    cout << "onSliderEvent speed is : " << e.value << endl;
    dvs.setTargetSpeed(e.value);
}

void ofApp::onTextInputEvent(ofxDatGuiTextInputEvent e)
{
    cout << "onTextInputEvent" << endl;
}

void ofApp::on2dPadEvent(ofxDatGui2dPadEvent e)
{
    cout << "on2dPadEvent" << endl;
}

void ofApp::onColorPickerEvent(ofxDatGuiColorPickerEvent e)
{
    cout << "onColorPickerEvent" << endl;
}

void ofApp::onMatrixEvent(ofxDatGuiMatrixEvent e)
{
	e.target->setRadioMode(true);
    //cout << "onMatrixEvent index " << e.child << " state " <<  e.enabled << endl;
	for(size_t i = 0; i < 6 ; i++){
		if(e.child == i){
			dvs.changeColor(i);
		}
	}
}

