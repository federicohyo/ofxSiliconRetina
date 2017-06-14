#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    dvs.setup();
}

//--------------------------------------------------------------
void ofApp::update(){
    dvs.update();
    //dvs.updateBAFilter();
    //dvs.updateImageGenerator();
    stats.update();
}

//--------------------------------------------------------------
void ofApp::draw(){
    dvs.drawFrames();
    //dvs.drawImageGenerator();
    dvs.drawSpikes();
    if(dvs.statsStatus){
        stats.draw();
    }
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
    if (key == 's') {
        dvs.changeStats(); //enable/disable stats
    }
    if (key == 'n'){ //load file from disk
        dvs.loadFile();
    }
    if (key == 'l'){ //connect to device
        dvs.tryLive();
    }
    if (key == '+') {
    	float tgc = dvs.getTargetSpeed();
        float change = tgc*0.1;
    	if(tgc+change < LONG_MAX){
    		dvs.changeTargetSpeed(+change); //faster
        }
    }
    if (key == '-') {
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
