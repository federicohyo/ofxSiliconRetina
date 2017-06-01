#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    dvs.setup();
    
    ofSetBackgroundColor(255);
    
    fbo.allocate(ofGetWidth(),ofGetHeight());
    fboCam.allocate(dvs.sizeX,dvs.sizeY);
    
    // webcam.setup(640,480);
}

//--------------------------------------------------------------
void ofApp::update(){
    
    dvs.update();
    
    mesh.clear();
    vector<polarity> packets = dvs.getPolarity();
    for(int i=0;i < packets.size();i++) {
        mesh.addVertex(ofVec3f(ofMap(packets[i].pos.x,0,dvs.sizeY,0,fbo.getWidth()),ofMap(packets[i].pos.y,0,dvs.sizeY,0,fbo.getHeight()), 0));
        mesh.addTexCoord(ofVec2f(packets[i].pos.x, packets[i].pos.y));
        if(packets[i].pol){
            mesh.addColor(ofColor(255,0,0));
        }else{
            mesh.addColor(ofColor(0,255,0));
        }
    }
    
}

//--------------------------------------------------------------
void ofApp::draw(){
    //    dvs.draw();
    
    fbo.begin();
    ofClear(0,0,0,255);
    
    mesh.setMode(OF_PRIMITIVE_POINTS);
    //  mesh.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);
    glPointSize(3);
    mesh.drawWireframe();
    fbo.end();
    
    //fbo.draw(0,0,ofGetWidth(),ofGetHeight());
    
    cam.begin();
    ofPushMatrix();
    ofTranslate(-ofGetWidth()/2, -ofGetHeight()/2);
    mesh.draw();
    ofPopMatrix();
    cam.end();
    
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if(key == 'f') {
        ofToggleFullscreen();
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
    if (key == 's') {
        dvs.changeRecordingStatus(); //enable/disable recording
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
