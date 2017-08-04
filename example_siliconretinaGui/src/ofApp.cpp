#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    dvs.setup();
    
    //GUI
    int x = 0;
    int y = 0;
    ofSetWindowPosition(0, 0);
    f1 = new ofxDatGuiFolder("Control", ofColor::fromHex(0xFFD00B));
    f1->addBreak();
    f1->addFRM();
    f1->addBreak();
    f1->addSlider("1/speed", 0, 2, dvs.targetSpeed);
    myTextTimer = f1->addTextInput("TIME", dvs.timeString);
    myTempReader = f1->addTextInput("IMU TEMPERATURE", to_string((int)(dvs.imuTemp)));
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
	f1->addToggle("Draw IMU", false);
    f1->addMatrix("3D Time", 4, true);
    f1->addToggle("Pointer", false);
    f1->addToggle("Raw Spikes", true);
    f1->addToggle("DVS Image Gen", false);
    f1->addSlider("BA Filter dt", 1, 100000, dvs.BAdeltaT);
    f1->addSlider("DVS Integration", 1, 100, dvs.fsint);
    f1->addSlider("DVS Image Gen", 1, 20000, dvs.numSpikes);
    //myIMU = f1->addValuePlotter("IMU", 0, 1);
    f1->setPosition(x, y);
    f1->expand();
    f1->onButtonEvent(this, &ofApp::onButtonEvent);
    f1->onToggleEvent(this, &ofApp::onToggleEvent);
    f1->onSliderEvent(this, &ofApp::onSliderEvent);
    f1->onMatrixEvent(this, &ofApp::onMatrixEvent);
    f1->onTextInputEvent(this, &ofApp::onTextInputEvent);

    numPaused = 0;
    numPausedRec = 0;
    
    // alpha blend
    //glEnable(GL_BLEND);
    //glBlendFunc( GL_DST_ALPHA, GL_ONE_MINUS_DST_ALPHA );
}

//--------------------------------------------------------------
void ofApp::update(){
    dvs.update();
    dvs.updateBAFilter();
    dvs.updateImageGenerator();

    //GUI
    f1->update();
    myTextTimer->setText(dvs.timeString);
    //float val = ofRandom(0, 1);
    //cout << val << endl;
    //myIMU->setValue(val);
    myTempReader->setText(to_string((int)(dvs.imuTemp)));
}

//--------------------------------------------------------------
void ofApp::draw(){

    dvs.draw();

    //GUI
    if(drawGui){
        f1->draw();
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if (key == 'c') {
        changeDrawGui();
    }
}

void ofApp::changeDrawGui(){
    if(drawGui){
        drawGui = false;
    }else{
        drawGui = true;
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
		numPausedRec++;
		if((numPausedRec % 2) == 0){
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
    if(e.target->getLabel() == "APS"){
    	dvs.changeAps();
    }else if(e.target->getLabel() == "DVS"){
    	dvs.changeDvs();
    }else if(e.target->getLabel() == "IMU"){
    	dvs.changeImu();
    }else if (e.target->getLabel() == "DVS Image Gen"){
        dvs.setDrawImageGen(e.target->getChecked());
    }else if(e.target->getLabel() == "Raw Spikes"){
        dvs.setDrawSpikes(e.target->getChecked());
    }else if(e.target->getLabel() == "Pointer"){
        dvs.setPointer(e.target->getChecked());
    }else if(e.target->getLabel() == "Draw IMU"){
        dvs.setDrawImu(e.target->getChecked());
    }

}

void ofApp::onSliderEvent(ofxDatGuiSliderEvent e)
{
    if(e.target->getLabel() == "1/speed"){
        cout << "onSliderEvent speed is : " << e.value << endl;
        dvs.setTargetSpeed(e.value);
    }else if(e.target->getLabel() == "DVS Integration"){
        cout << "Integration fsint is : " << e.value << endl;
        dvs.changeFSInt(e.value);
    }else if( e.target->getLabel() == "BA Filter dt"){
        cout << "BackGround Filter dt : " << e.value << endl;
        dvs.changeBAdeltat(e.value);
    }else if( e.target->getLabel() == "DVS Image Gen"){
        cout << "Accumulation value : " << e.value << endl;
        dvs.setImageAccumulatorSpikes(e.value);
    }
    dvs.myCam.reset(); // no mesh turning when using GUI
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
    
    if( e.target->getLabel() == "3D Time"){
        e.target->setRadioMode(true);
        for(size_t i = 0; i < 4 ; i++){
            if(e.child == i){
                dvs.set3DTime(i);
            }
        }
    }else if(e.target->getLabel() == "DVS Color"){
        e.target->setRadioMode(true);
        for(size_t i = 0; i < 6 ; i++){
            if(e.child == i){
                dvs.changeColor(i);
            }
        }
    }
}

