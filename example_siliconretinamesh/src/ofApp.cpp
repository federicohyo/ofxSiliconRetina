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
    ofSetVerticalSync(true);

    tmp = 0;
    started = false;
    m = 4;
    nus = 10000;
    
    //disable aps
    dvs.changeAps();
}

//--------------------------------------------------------------
void ofApp::update(){
    
    stats.update();
    
    dvs.update();
    
    mesh.clear();
    vector<polarity> packets = dvs.getPolarity();
    for(int i=0;i < packets.size();i++) {
        //long tdiff = (int) ofRandom(1000) % 1000;//packets[i].timestamp - dvs.ofxLastTs;
        long tdiff = 0;
        if( packets[i].timestamp < tmp){
            ofLog(OF_LOG_NOTICE, "Detected lower timestamp.. ");
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
            mesh.clear();
            tdiff = 0;
            tmp = packets[i].timestamp;//tmp-nus;
        }
        long timeus = tdiff>>m;
        mesh.addVertex(ofVec3f(ofMap(packets[i].pos.x,0,dvs.sizeX,0,fbo.getWidth()),ofMap(packets[i].pos.y,0,dvs.sizeY,0,fbo.getHeight()), timeus));
        mesh.addTexCoord(ofVec2f(packets[i].pos.x, packets[i].pos.y));
        //int alphaus = (int)ceil(((float)tdiff/(float)nus)*256);
        if(packets[i].pol){
            mesh.addColor(ofColor(255,0,0));
        }else{
            mesh.addColor(ofColor(0,255,0));
        }
    }
    
    
}

//--------------------------------------------------------------
void ofApp::draw(){

    ofClear(0,0,0,255);
    cam.begin();
    mesh.setMode(OF_PRIMITIVE_POINTS);
    glPointSize(3);
    mesh.drawWireframe();
    ofPushMatrix();
    mesh.draw();
    ofPopMatrix();
    cam.end();
    
    // stats
    if(dvs.statsStatus){
        stats.draw();
    }
    
    // Nearest Vertex
    int n = mesh.getNumVertices();
    float nearestDistance = 0;
    ofVec3f nearestVertex;
    ofVec3f nearestVertexCam;
    int nearestIndex = 0;
    ofVec2f mouse(mouseX, mouseY);
    for(int i = 0; i < n; i++) {
        ofVec3f cur = cam.worldToScreen(mesh.getVertex(i));
        ofVec3f camCur = mesh.getVertex(i);
        float distance = cur.distance(mouse);
        if(i == 0 || distance < nearestDistance) {
            nearestDistance = distance;
            nearestVertex = cur;
            nearestVertexCam = camCur;
            nearestIndex = i;
        }
    }
    
    ofSetColor(ofColor::gray);
    ofDrawLine(nearestVertex, mouse);
    
    ofNoFill();
    ofSetColor(ofColor::yellow);
    ofSetLineWidth(2);
    ofDrawCircle(nearestVertex, 4);
    ofSetLineWidth(1);
    
    ofVec2f offset(10, -10);
    ofVec2f origXY = ofVec2f(ofMap(nearestVertexCam.x,0,fbo.getWidth(),0,dvs.sizeX),ofMap(nearestVertexCam.y,0,fbo.getHeight(),0,dvs.sizeY));
    long zconv;
    if(m > 0){
        zconv = (long)(nearestVertexCam.z)<<m;
    }else{
        zconv = (long)nearestVertexCam.z;
    }
    string infos = "x:" + ofToString(origXY.x) + " y:" + ofToString(origXY.y) + " z: "+ofToString(zconv)+" us";
    ofDrawBitmapStringHighlight(infos, mouse + offset);
    
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if(key == 'f') {
        ofToggleFullscreen();
    }
    if (key == 'c') {
        dvs.loopColor(); //change color for dvs events
    }
    //if (key == 'a') {
    //    dvs.changeAps(); //enable/disable aps
    //}
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
        dvs.changeStats(); //enable/disable stats
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
    if (key == 'm') {
        if(m < 8){
            m += 2;
        }else{
            ofLog(OF_LOG_NOTICE, "max value reached %lu", m);
        }
    }
    if (key == 'k') {
        if(m >= 2){
            m -= 2;
        }else{
            ofLog(OF_LOG_NOTICE, "min value reached %lu", m);
        }
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
