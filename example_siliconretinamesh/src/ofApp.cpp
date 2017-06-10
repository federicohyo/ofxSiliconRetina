#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    dvs.setup();
    
    ofSetVerticalSync(true);

    ofSetBackgroundColor(255);
    
    fbo.allocate(ofGetWidth(),ofGetHeight());
    fboCam.allocate(dvs.sizeX,dvs.sizeY);

    mesh.setMode(OF_PRIMITIVE_POINTS);
    
    // webcam.setup(640,480);
    ofEnableDepthTest();
    glEnable(GL_POINT_SMOOTH); // use circular points instead of square points
    glPointSize(3);
    
    tmp = 0;
    started = false;
    m = 4;
    nus = 10000;
}

//--------------------------------------------------------------
void ofApp::update(){
    
    dvs.update();
    
    mesh.clear();
    vector<polarity> packets = dvs.getPolarity();
    for(int i=0;i < packets.size();i++) {
        //long tdiff = (int) ofRandom(1000) % 1000;//packets[i].timestamp - dvs.ofxLastTs;
        long tdiff = 0;
        if( packets[i].timestamp < tmp){
            tmp = packets[i].timestamp;
        }
        if(started == false){
            tdiff = 0;
            tmp = packets[i].timestamp;
            started = true;
        }else{
            tdiff = packets[i].timestamp - tmp;
        }
        if(tdiff > nus){
            tmp = packets[i].timestamp;
        }
        //cout << "current ts "<< packets[i].timestamp << " start ts " << tmp << "tdiff" << tdiff <<endl;
        mesh.addVertex(ofVec3f(ofMap(packets[i].pos.x,0,dvs.sizeY,0,fbo.getWidth()),ofMap(packets[i].pos.y,0,dvs.sizeY,0,fbo.getHeight()), tdiff>>m));
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
    if (key == 'r') {
        dvs.changeRecordingStatus(); //enable/disable recording
    }
    if (key == 's') {
        dvs.changeRecordingStatus(); //enable/disable recording
    }
    if (key == 'n'){ //load file from disk
        dvs.loadFile();
    }
    if (key == 'p'){ //load file from disk
        dvs.changePause();
    }
    if (key == 'l'){ //connect to device
        dvs.tryLive();
    }
    if (key == '+') {
        dvs.changeTargetSpeed(+500); //faster
    }
    if (key == '-') {
        dvs.changeTargetSpeed(-500); //slower
    }
    if (key == 'm') {
        m += 2;
    }
    if (key == 'k') {
        m -= 2;
    }
    if (key == 'j') {
        int perc = (nus / 100) * 10;
        nus += perc;
    }
    if (key == 'h') {
        int perc = (nus / 100) * 10;
        nus -= perc;
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
