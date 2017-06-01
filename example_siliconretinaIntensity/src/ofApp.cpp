#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    // shader
    ofSetFrameRate(60);
    ofEnableAlphaBlending();
    ofSetWindowShape(dvs.sizeX*mulSize, dvs.sizeX*mulSize);
    ofSetWindowPosition(1, 1);
    events.allocate(dvs.sizeY*mulSize,dvs.sizeY*mulSize);
    // dvs
    dvs.setup();
}

//--------------------------------------------------------------
void ofApp::update(){
    dvs.update();
    
    events.begin();
    ofFill();
    ofSetColor(ofNoise( ofGetFrameNum() ) * 255 * 5, 255);
    dvs.drawSpikes();
    events.end();
    events.update();
}

//--------------------------------------------------------------
void ofApp::draw(){

    ofBackground(128);
    ofSetColor(255,255);
    events.draw(0,0);
    ofDrawBitmapString("ofxEvents ( dumping = " + ofToString(events.dumping) + " )", 15,15);
    
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    // Tweak dumping
    if (key == OF_KEY_UP){
        events.dumping += 0.005;
    } else if ( key == OF_KEY_DOWN){
        events.dumping -= 0.005;
    }
    
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
    //events.dumping = ofMap(y, 0, ofGetHeight(), 0.9, 1.0, true);
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
