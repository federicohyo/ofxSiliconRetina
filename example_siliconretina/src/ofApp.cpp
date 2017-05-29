#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    dvs.setup();
}

//--------------------------------------------------------------
void ofApp::update(){
    dvs.update();
    stats.update();
}

//--------------------------------------------------------------
void ofApp::draw(){
    dvs.drawFrames();
    dvs.drawSpikes();
    stats.draw();
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
