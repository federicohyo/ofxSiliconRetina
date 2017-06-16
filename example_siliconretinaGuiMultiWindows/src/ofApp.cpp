#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	dvs.setup();
}

//--------------------------------------------------------------
void ofApp::setupGui(){

	controls.setName("controls");
	controls.add(targetSpeedGui.set("1/speed",0.0001,3,0.001));
	controls.add(APS.set("APS", true));
	controls.add(DVS.set("DVS", true));
	controls.add(IMU.set("IMU", true));
	controls.add(dvscolor.set("DVS Color",0,7,0));
	controls.add(clear.set("Clear", false));
	controls.add(pause.set("Pause", false));
	controls.add(recording.set("Recording", false));
	controls.add(loadRec.set("Load File", false));
	controls.add(livemode.set("Live Mode", true));
	controls.add(baFilterDt.set("BA Filter dt", 0, 100000, dvs.BAdeltaT));
	controls.add(dvsIntegration.set("DVS Integration dt",0, 50, dvs.fsint));

	//controls.add(color.set("color",100,ofColor(0,0),255));
	gui.setup(controls);
	ofSetBackgroundColor(0);
}

//--------------------------------------------------------------
void ofApp::update(){

	dvs.update();
	updateFromGui();
}

void ofApp::updateFromGui(){
	dvs.targetSpeed = targetSpeedGui;
	if(IMU.get() !=  dvs.imuStatus){
		dvs.changeImu();
	}
	if(DVS.get() !=  dvs.dvsStatus){
		dvs.changeDvs();
	}
	if(APS.get() !=  dvs.apsStatus){
		dvs.changeAps();
	}
	dvs.changeColor(dvscolor);
	if(clear){
		dvs.clearDraw();
		clear.set("Clear", false);
	}
	if(pause.get() != dvs.paused){
		dvs.changePause();
	}
	if(recording.get() != dvs.isRecording){
		dvs.changeRecordingStatus();
	}
	if(loadRec){
		dvs.loadFile();
		dvs.setPause(false);
		dvs.liveInput = false;
		livemode = false;
		livemode.set("Live Mode", false);
		loadRec.set("Load File", false);
	}
	if(livemode != dvs.liveInput){
		dvs.tryLive();
		livemode.set("Live Mode", livemode);
	}
	if(baFilterDt.get() != dvs.BAdeltaT ){
		dvs.changeBAdeltat(baFilterDt);
	}
	if(dvsIntegration.get() != dvs.fsint){
		dvs.changeFSInt(dvsIntegration);
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	dvs.drawFrames();
	dvs.drawSpikes();

}

//--------------------------------------------------------------
void ofApp::drawGui(ofEventArgs & args){
	gui.draw();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

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
