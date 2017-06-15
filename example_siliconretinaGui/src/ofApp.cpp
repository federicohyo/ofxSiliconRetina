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
    f1->addSlider("1/speed", 0, 3, 0.3);
    f1->setPosition(x, y);
    f1->expand();

    f1->onButtonEvent(this, &ofApp::onButtonEvent);
    f1->onToggleEvent(this, &ofApp::onToggleEvent);
    f1->onSliderEvent(this, &ofApp::onSliderEvent);
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
    if (key == 'c') {
        dvs.loopColor(); //change color for dvs events
    }
    if (key == 'a') {
        dvs.changeAps(); //enable/disable aps
    }
    if (key == 'd') {
        dvs.changeDvs(); //enable/disable dvs
    }
    if (key == 'i') {
        dvs.changeImu(); //enable/disable imu
    }
    if (key == 'p'){ //load file from disk
        dvs.changePause();
    }
    if (key == 'r') {
        dvs.changeRecordingStatus(); //enable/disable recording
    }
    if (key == 'n'){ //load file from disk
        dvs.loadFile();
    }
    if (key == 'l'){ //connect to device
        dvs.tryLive();
    }
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
    cout << "onButtonEvent" << endl;
}

void ofApp::onToggleEvent(ofxDatGuiToggleEvent e)
{
    cout << "onToggleEvent " << e.checked << endl;
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
    cout << "onMatrixEvent" << endl;
}

