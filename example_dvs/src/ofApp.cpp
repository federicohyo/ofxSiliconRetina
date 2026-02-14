#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    dvs.setupCore();
}

//--------------------------------------------------------------
void ofApp::update(){
    dvs.update();
}

//--------------------------------------------------------------
void ofApp::draw(){
    dvs.drawViewer();
}

//--------------------------------------------------------------
void ofApp::exit(){
    dvs.exit();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    dvs.keyPressed(key);
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
    dvs.mousePressed(x, y, button);
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
