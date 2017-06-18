//
//  ofxDVS.cpp
//  ofxDVS
//
//  Created by Federico Corradi on 19.05.17.
//
//

#include "ofxDVS.hpp"

using namespace std::placeholders;

//--------------------------------------------------------------
ofxDVS::ofxDVS() {
    
    doChangePath = false;
    header_skipped = false;
}

//--------------------------------------------------------------
void ofxDVS::setup() {
    
    //thread_alpha.startThread();   // start usb thread
    thread.startThread();   // start usb thread
    /*thread.getPocoThread().setPriority(Poco::Thread::PRIO_HIGHEST);
    cout << thread.getPocoThread().getPriority() << " min"
    	<<thread.getPocoThread().getMinOSPriority() <<
		" max" << thread.getPocoThread().getMaxOSPriority() << endl;
     */

    // default behaviour is to start live mode
    // after 1 seconds of non finding the device we start in file mode
    ofResetElapsedTimeCounter();
    uint64_t t0 = ofGetElapsedTimeMicros();
    
    // get camera size after ready
    LOCK_CHECK:
    nanosleep((const struct timespec[]){{0, 50000000L}}, NULL);
    thread.lock();
    if(thread.deviceReady!=true && thread.fileInputReady!=true){
    	thread.unlock();
        uint64_t t1 = ofGetElapsedTimeMicros();
        //cout << ns << endl;
        if( (t1-t0) > 10000){
            ofLog(OF_LOG_NOTICE, "starting in file mode.");
            ofFileDialogResult result = ofSystemLoadDialog("Load aedat3.1 file");
            if(result.bSuccess) {
                path = result.getPath();
                // load your file at `path`
                changePath();
            }
        }
    	goto LOCK_CHECK;
    }
    
    // viewer started in live or file mode
	sizeX = thread.sizeX;
	sizeY = thread.sizeY;
    chipId = thread.chipId;
	thread.unlock();

	fsint = 2;
    /*thread_alpha.lock();
    thread.lock();
    thread_alpha.sizeX = thread.sizeX;
    thread_alpha.sizeY = thread.sizeY;
    thread_alpha.fsint = fsint = 2;
    thread_alpha.init = true;
    thread.unlock();
    thread_alpha.unlock();*/

	// init framebuffer
    ofSetVerticalSync(true);
    ofSetBackgroundColor(255);
    
    //
    fbo.allocate(sizeX, sizeY, GL_RGBA32F);
    tex = &fbo.getTexture();
    mesh.setMode(OF_PRIMITIVE_POINTS);
    glEnable(GL_POINT_SMOOTH); // use circular points instead of square points
    
    // init spike colors
    initSpikeColors();

    // start the thread
    initThreadVariables();

    // init spikefeature/imagegenerator
    initImageGenerator();

    // init baFilterState
    initBAfilter();
    
    // init alpha map
    initVisualizerMap();
    
    // reset timestamp
    ofResetElapsedTimeCounter();
    ofxLastTs = 0;
    targetSpeed = 0.01; // real_time
    paused = false;
    started = 0;
    isStarted = false;
    microseconds = 0;
    seconds = 0;
    minutes = 0;
    hours = 0;
    doDrawSpikes = true;
    
    // mesh
    tmp = 0;
    m = 0;
    nus = 10000;
    
}

void ofxDVS::initThreadVariables(){
    apsStatus = true;       // enable aps
    dvsStatus = true;       // enable dvs
    imuStatus = true;       // enable imu
    maxContainerQueued = 1000; // at most accumulates 100 packaetcontainers before dropping
    packetContainer = NULL;
    isRecording = false;
}

void ofxDVS::tryLive(){
    thread.lock();
    // free al memory
    if(thread.fileInput){
        // clean memory
        vector<polarity>().swap(packetsPolarity);
        packetsPolarity.clear();
        packetsPolarity.shrink_to_fit();
        vector<frame>().swap(packetsFrames);
        packetsFrames.clear();
        packetsFrames.shrink_to_fit();
        vector<imu6>().swap(packetsImu6);
        packetsImu6.clear();
        packetsImu6.shrink_to_fit();
        header_skipped  = false;
        thread.fileInput = false;
        thread.header_skipped = header_skipped;
        for(int i=0; i<thread.container.size(); i++){
            packetContainer = thread.container.back();
            thread.container.pop_back();
            caerEventPacketContainerFree(packetContainer);
            thread.container.clear();
            thread.container.shrink_to_fit();
        }
    }
    thread.header_skipped  = true;
    thread.fileInput = false;
    thread.liveInput = true;
    liveInput = thread.liveInput;
    thread.unlock();
}

//--------------------------------------------------------------
void ofxDVS::changePath(){
    thread.lock();
    thread.doChangePath = true;
    ofLog(OF_LOG_NOTICE, path);
    // update file path
    thread.path = path;
    // clean memory
    vector<polarity>().swap(packetsPolarity);
    packetsPolarity.clear();
    packetsPolarity.shrink_to_fit();
    vector<frame>().swap(packetsFrames);
    packetsFrames.clear();
    packetsFrames.shrink_to_fit();
    vector<imu6>().swap(packetsImu6);
    packetsImu6.clear();
    packetsImu6.shrink_to_fit();
    thread.istreamf.close();
    thread.header_skipped  = false;
    thread.fileInput = true;
    thread.header_skipped = header_skipped;
    for(int i=0; i<thread.container.size(); i++){
        packetContainer = thread.container.back();
        thread.container.pop_back();
        caerEventPacketContainerFree(packetContainer);
        thread.container.clear();
        thread.container.shrink_to_fit();
    }
    liveInput = false;
    thread.unlock();
}


//--------------------------------------------------------------
void ofxDVS::openRecordingFile(){
    // File Recording Output
    //string path;
    path = getUserHomeDir();
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    char buffer [80];
    strftime (buffer,80,"ofxDVS_%Y-%m-%d-%I_%M_%S.aedat",now);
    string filename = path + "/" + buffer;
    myFile.open(filename, ios::out | ios::binary);
    writeHeaderFile();
}

//--------------------------------------------------------------
void ofxDVS::writeHeaderFile(){
    // Time
    struct tm currentTime;
    time_t currentTimeEpoch = time(NULL);
    localtime_r(&currentTimeEpoch, &currentTime);
    size_t currentTimeStringLength = 44;
    char currentTimeString[currentTimeStringLength + 1]; // + 1 for terminating NUL byte.
    strftime(currentTimeString, currentTimeStringLength + 1, "#Start-Time: %Y-%m-%d %H:%M:%S (TZ%z)\r\n", &currentTime);
    
    // Select Chip Id
    size_t sourceStringLength = (size_t) snprintf(NULL, 0, "#Source 0: %s\r\n",chipIDToName(chipId, false));
    char sourceString[sourceStringLength + 1];
    snprintf(sourceString, sourceStringLength + 1, "#Source 0: %s\r\n",chipIDToName(chipId, false));
    sourceString[sourceStringLength] = '\0';
    // Write Header
    myFile << "#!AER-DAT3.1\r\n";
    myFile << "#Format: RAW\r\n";
    myFile << sourceString;
    myFile << currentTimeString;
    myFile << "#!END-HEADER\r\n";
}

//--------------------------------------------------------------
void ofxDVS::changeRecordingStatus(){
    if(isRecording){
        isRecording = false;
        //close file
        myFile.close();
        ofLog(OF_LOG_NOTICE, "Stop recording\n");
    }else{
        //open file and write header
        openRecordingFile();
        isRecording = true;
        ofLog(OF_LOG_NOTICE, "Start recording\n");
    }
}

//--------------------------------------------------------------
void ofxDVS::initSpikeColors() {
    
    paletteSpike = 0;
    
    // set colors for spikes
    spkOnR[0] = 255;
    spkOnG[0] = 0;
    spkOnB[0] = 0;
    spkOffR[0] = 0;
    spkOffG[0] = 255;
    spkOffB[0] = 0;
    spkOnR[1] = 255;
    spkOnG[1] = 255;
    spkOnB[1] = 255;
    spkOffR[1] = 0;
    spkOffG[1] = 0;
    spkOffB[1] = 0;
    spkOnR[2] = 0;
    spkOnG[2] = 0;
    spkOnB[2] = 0;
    spkOffR[2] = 0;
    spkOffG[2] = 255;
    spkOffB[2] = 255;
    spkOnR[3] = 0;
    spkOnG[3] = 255;
    spkOnB[3] = 125;
    spkOffR[3] = 0;
    spkOffG[3] = 255;
    spkOffB[3] = 125;
    spkOnR[4] = 0;
    spkOnG[4] = 125;
    spkOnB[4] = 255;
    spkOffR[4] = 125;
    spkOffG[4] = 255;
    spkOffB[4] = 0;
    spkOnR[5] = 0;
    spkOnG[5] = 150;
    spkOnB[5] = 255;
    spkOffR[5] = 255;
    spkOffG[5] = 150;
    spkOffB[5] = 150;
    spkOnR[6] = 0;
    spkOnG[6] = 150;
    spkOnB[6] = 255;
    spkOffR[6] = 150;
    spkOffG[6] = 255;
    spkOffB[6] = 0;
    spkOnR[7] = 150;
    spkOnG[7] = 0;
    spkOnB[7] = 255;
    spkOffR[7] = 150;
    spkOffG[7] = 0;
    spkOffB[7] = 150;

    spkOnA = 255;
    spkOffA = 255;
}


//--------------------------------------------------------------
vector<polarity> ofxDVS::getPolarity() {
    return packetsPolarity;
}

//--------------------------------------------------------------
vector<frame> ofxDVS::getFrames() {
    return packetsFrames;
}

//--------------------------------------------------------------
void ofxDVS::clearDraw(){

    int clearCol = 0;
    if(paletteSpike == 0){
        clearCol = 0;
    }else if(paletteSpike == 1){
        clearCol = 127;
    }else if(paletteSpike == 2){
        clearCol = 150;
    }else if(paletteSpike == 3){
        clearCol = 0;
    }
    
	// make a black frame
	frame nuPackFrames;
	nuPackFrames.exposureStart = 0;
	nuPackFrames.exposureEnd = 0;
	nuPackFrames.lenghtX = sizeX;
	nuPackFrames.lenghtY = sizeY;
	nuPackFrames.positionX = 0;
	nuPackFrames.positionY = 0;
	nuPackFrames.singleFrame.allocate(nuPackFrames.lenghtX, nuPackFrames.lenghtY, OF_IMAGE_COLOR);
	for (int32_t y = 0; y < nuPackFrames.lenghtY; y++) {
		for (int32_t x = 0; x < nuPackFrames.lenghtX; x++) {
			ofColor color= ofColor(clearCol,clearCol,clearCol);
			nuPackFrames.singleFrame.setColor(x, y, color);
		}
	}
    nuPackFrames.valid = true;
    nuPackFrames.frameEnd = 0;
    nuPackFrames.frameStart = 0;
    packetsFrames.push_back(nuPackFrames);

}

//--------------------------------------------------------------
bool ofxDVS::organizeData(caerEventPacketContainer packetContainer){

	// every time here get targetSpeed us of data (fixed us playback)
	//long timeInterval = targetSpeed;
    if (packetContainer == NULL) {
        return(false); // Skip if nothing there.
    }
    
    int32_t packetNum = caerEventPacketContainerGetEventPacketsNumber(packetContainer);
    /*
    firstTs = caerEventPacketContainerGetLowestEventTimestamp(packetContainer);
    highestTs = caerEventPacketContainerGetHighestEventTimestamp(packetContainer);
	long current_file_dt = highestTs-firstTs;
	long current_ofx_dt = ofGetElapsedTimeMicros() - ofxLastTs;

	cout << "highestTs " << highestTs << " FirstTS " << firstTs << " ofxLastTs " << ofxLastTs << endl;
	cout << " current_file_dt " <<  current_file_dt << " current_ofx_dt " << current_ofx_dt << endl;
     */

    for (int32_t i = 0; i < packetNum; i++) {
        
        polarity nuPack;
        frame nuPackFrames;
        imu6 nuPackImu6;
        
        caerEventPacketHeader packetHeader = caerEventPacketContainerGetEventPacket(packetContainer, i);
        if (packetHeader == NULL) {
            //ofLog(OF_LOG_WARNING,"Packet %d is empty (not present).\n", i);
            continue; // Skip if nothing there.
        }
        
        int type = caerEventPacketHeaderGetEventType(packetHeader);
        long lastTs = 0;

        //ofLog(OF_LOG_WARNING,"Packet %d of type %d -> size is %d.\n", i, caerEventPacketHeaderGetEventType(packetHeader),
        //           caerEventPacketHeaderGetEventNumber(packetHeader));
        
        if (type == IMU6_EVENT && imuStatus) {
            
            packetsImu6.clear();
            packetsImu6.shrink_to_fit();
            
            caerIMU6EventPacket imu6 = (caerIMU6EventPacket) packetHeader;
            
            float accelX = 0, accelY = 0, accelZ = 0;
            float gyroX = 0, gyroY = 0, gyroZ = 0;
            
            CAER_IMU6_ITERATOR_VALID_START((caerIMU6EventPacket) imu6)
            accelX = caerIMU6EventGetAccelX(caerIMU6IteratorElement);
            accelY = caerIMU6EventGetAccelY(caerIMU6IteratorElement);
            accelZ = caerIMU6EventGetAccelZ(caerIMU6IteratorElement);
            gyroX = caerIMU6EventGetGyroX(caerIMU6IteratorElement);
            gyroY = caerIMU6EventGetGyroY(caerIMU6IteratorElement);
            gyroZ = caerIMU6EventGetGyroZ(caerIMU6IteratorElement);

            nuPackImu6.accel.set(accelX,accelY,accelZ);
            nuPackImu6.gyro.set(gyroX,gyroY,gyroZ);
            nuPackImu6.timestamp = caerIMU6EventGetTimestamp(caerIMU6IteratorElement);
            nuPackImu6.valid = true ;
            packetsImu6.push_back(nuPackImu6);

            if(nuPackImu6.timestamp > lastTs){
                lastTs = nuPackImu6.timestamp;
            }

            CAER_IMU6_ITERATOR_VALID_END
            
        }
        if (type == POLARITY_EVENT  && dvsStatus) {
            
            packetsPolarity.clear();
            packetsPolarity.shrink_to_fit();
            
            caerPolarityEventPacket polarity = (caerPolarityEventPacket) packetHeader;
            
            CAER_POLARITY_ITERATOR_VALID_START(polarity)
            nuPack.timestamp = caerPolarityEventGetTimestamp64(caerPolarityIteratorElement, polarity);
            
            nuPack.pos.x = caerPolarityEventGetX(caerPolarityIteratorElement);
            nuPack.pos.y = caerPolarityEventGetY(caerPolarityIteratorElement);
            
            nuPack.pol = caerPolarityEventGetPolarity(caerPolarityIteratorElement);
            
            nuPack.valid = true;
            packetsPolarity.push_back(nuPack);

            if(nuPack.timestamp > lastTs){
                lastTs = nuPack.timestamp;
            }

            CAER_POLARITY_ITERATOR_VALID_END
        }
        if (type == FRAME_EVENT && apsStatus){
            // first time we get in here
            // otherwise we do not clear packetframes
            // and we keep the last one
            
            packetsFrames.clear();
            packetsFrames.shrink_to_fit();

            caerFrameEventPacket frame = (caerFrameEventPacket) packetHeader;
            
            CAER_FRAME_ITERATOR_VALID_START(frame)
            nuPackFrames.exposureStart = caerFrameEventGetTSStartOfExposure(caerFrameIteratorElement);
            nuPackFrames.exposureEnd = caerFrameEventGetTSEndOfExposure(caerFrameIteratorElement);
            // Use frame sizes to correctly support small ROI frames.
            nuPackFrames.lenghtX = caerFrameEventGetLengthX(caerFrameIteratorElement);
            nuPackFrames.lenghtY = caerFrameEventGetLengthY(caerFrameIteratorElement);
            nuPackFrames.positionX = caerFrameEventGetPositionX(caerFrameIteratorElement);
            nuPackFrames.positionY = caerFrameEventGetPositionY(caerFrameIteratorElement);
            nuPackFrames.frameChannels = caerFrameEventGetChannelNumber(caerFrameIteratorElement);
            
            nuPackFrames.singleFrame.allocate(nuPackFrames.lenghtX, nuPackFrames.lenghtY, OF_IMAGE_COLOR);
            for (int32_t y = 0; y < nuPackFrames.lenghtY; y++) {
                for (int32_t x = 0; x < nuPackFrames.lenghtX; x++) {
                    
                    switch (nuPackFrames.frameChannels) {
                            
                        case GRAYSCALE: {
                            int nuCol = U8T(caerFrameEventGetPixelUnsafe(caerFrameIteratorElement, x, y) >> 8);
                            
                            ofColor color= ofColor(nuCol,nuCol,nuCol);
                            nuPackFrames.singleFrame.setColor(x, y, color);
                            
                            break;
                        }
                        case RGB: {
                            int nuColR = U8T(caerFrameEventGetPixelForChannelUnsafe(caerFrameIteratorElement, x, y, 0) >> 8);
                            int nuColG = U8T(caerFrameEventGetPixelForChannelUnsafe(caerFrameIteratorElement, x, y, 1) >> 8);
                            int nuColB = U8T(caerFrameEventGetPixelForChannelUnsafe(caerFrameIteratorElement, x, y, 2) >> 8);
                            
                            ofColor color= ofColor(nuColR,nuColB,nuColB);
                            nuPackFrames.singleFrame.setColor(x, y, color);
                            
                            break;
                        }
                        case RGBA:
                        default: {
                            int nuColR = U8T(caerFrameEventGetPixelForChannelUnsafe(caerFrameIteratorElement, x, y, 0) >> 8);
                            int nuColG = U8T(caerFrameEventGetPixelForChannelUnsafe(caerFrameIteratorElement, x, y, 1) >> 8);
                            int nuColB = U8T(caerFrameEventGetPixelForChannelUnsafe(caerFrameIteratorElement, x, y, 2) >> 8);
                            int nuColA = U8T(caerFrameEventGetPixelForChannelUnsafe(caerFrameIteratorElement, x, y, 3) >> 8);
                            
                            ofColor color= ofColor(nuColR,nuColB,nuColB,nuColA);
                            nuPackFrames.singleFrame.setColor(x, y, color);
                            break;
                        }
                            
                    }
                    
                }
            }
            nuPackFrames.valid = true;
            nuPackFrames.frameEnd = caerFrameEventGetTSEndOfExposure(caerFrameIteratorElement);
            nuPackFrames.frameStart = caerFrameEventGetTSStartOfExposure(caerFrameIteratorElement);

            packetsFrames.push_back(nuPackFrames);
            nuPackFrames.singleFrame.clear();
            if(nuPackFrames.frameEnd > lastTs){
                lastTs = nuPackFrames.frameEnd;
            }

            CAER_FRAME_ITERATOR_VALID_END
        }
    }
    
	//keep track of last commit
    ofxLastTs = ofGetElapsedTimeMicros();
    return(true);

}

//--------------------------------------------------------------
void ofxDVS::update() {
    
    if(paused == false){
        // Copy data from usbThread
        thread.lock();
        for(int i=0; i<thread.container.size(); i++){
            packetContainer = thread.container[i];

            long firstTs = caerEventPacketContainerGetLowestEventTimestamp(packetContainer);
            long highestTs = caerEventPacketContainerGetHighestEventTimestamp(packetContainer);
            long current_file_dt = highestTs-firstTs;
            long current_ofx_dt = ofGetElapsedTimeMicros() - ofxLastTs;
            

            bool delpc = false;
            if(targetSpeed <= 0){
            	targetSpeed = 0.0000001;
            }
            if( current_file_dt <  (float)current_ofx_dt/targetSpeed){
                //cout << "highestTs " << highestTs << " FirstTS " << firstTs << " ofxLastTs " << ofxLastTs << endl;
                //cout << " current_file_dt " <<  current_file_dt << " current_ofx_dt " << current_ofx_dt << endl;
                delpc = organizeData(packetContainer);
                
                // update time information
                long ts = caerEventPacketContainerGetHighestEventTimestamp(packetContainer);
                if(isStarted == false){
                    started = ts;
                    isStarted = true;
                }
                unsigned long current =  ts - started;
                microseconds = current - (minutes*60)*1e6 - seconds*1e6;
                minutes = (current / 60e6);
                seconds = ( ((int)current % (int)60e6) / 1e6);
                hours =  ((((int)current % (int)60e6) / 1e6) / 1e6);
                //cout << hours << ":" << minutes << ":" << seconds << ":" << microseconds << endl;
                sprintf(timeString, " %02lu:%02lu:%02lu:%04lu", hours, minutes, seconds, microseconds);
            }else{
                // no deal
                if(i>0){
                    i = i-1;
                }
            }
            // recording status
            if(isRecording){
                // order packet containers in time - file format aedat 3.1 standard -
                size_t currPacketContainerSize = (size_t) caerEventPacketContainerGetEventPacketsNumber(packetContainer);
                qsort(packetContainer->eventPackets, currPacketContainerSize, sizeof(caerEventPacketHeader),
                      &packetsFirstTimestampThenTypeCmp);
                // save to files
                int32_t packetNum = caerEventPacketContainerGetEventPacketsNumber(packetContainer);
                for (int32_t i = 0; i < packetNum; i++) {
                    caerEventPacketHeader packetHeader = caerEventPacketContainerGetEventPacket(packetContainer, i);
                    if(packetHeader == NULL){
                        continue;
                    }
                    caerEventPacketHeaderSetEventCapacity(packetHeader, caerEventPacketHeaderGetEventNumber(packetHeader));
                    size_t sizePacket = caerEventPacketGetSize(packetHeader);
                    caerEventPacketHeaderSetEventSource(packetHeader, caerEventPacketHeaderGetEventSource(packetHeader));
                    myFile.write((char*)packetHeader, sizePacket);

                }
            }
            // free all packet containers here
            if(delpc){
            	caerEventPacketContainerFree(packetContainer);
            	thread.container.erase( thread.container.begin()+i );
            }
            thread.doLoad = delpc;
        }
        //thread.container.clear();
        //thread.container.shrink_to_fit();
        // done with the resource
        thread.unlock();
        
        // check how fast we are going, if we are too slow, drop some data
        /*thread.lock();
        if(thread.container.size() > maxContainerQueued){
            ofLog(OF_LOG_WARNING, "Visualization is too slow, dropping events to keep real-time.");
            packetContainer = thread.container.back(); // this is a pointer
            thread.container.pop_back();
            caerEventPacketContainerFree(packetContainer);
            thread.container.clear();
            thread.container.shrink_to_fit();
        }
        thread.unlock();*/
        
    }else{
        // we are paused..
        thread.lock();
        // in live mode .. just trow away
        for(int i=0; i<thread.container.size(); i++){
            packetContainer = thread.container.back(); // this is a pointer
            thread.container.pop_back();
            // free all packet containers here
            caerEventPacketContainerFree(packetContainer);
        }
        // done with the resource
        thread.unlock();
    }
}

//--------------------------------------------------------------
void ofxDVS::loopColor() {
    if(paletteSpike < 2){
        paletteSpike += 1;
    }else{
        paletteSpike = 0;
    }
}

//--------------------------------------------------------------
void ofxDVS::changeColor(int i) {
    if(paletteSpike < 7){
        paletteSpike = i;
    }else{
        paletteSpike = 0;
    }
}

void ofxDVS::changeFSInt(float i){
    if(fsint == 0){
        fsint = 0.00001;
    }else{
        fsint = i;
    }
    /*thread_alpha.lock();
    thread_alpha.fsint  = fsint;
    thread_alpha.unlock();*/
}

//--------------------------------------------------------------
void ofxDVS::changeBAdeltat(float i){
    BAdeltaT = i;
}

//--------------------------------------------------------------
void ofxDVS::drawMouseDistanceToSpikes(){
    
    //ofTranslate(ofPoint(-ofGetWidth()/2,-ofGetHeight()/2));
    // Nearest Vertex
    int n = mesh.getNumVertices();
    float nearestDistance = 0;
    ofVec3f nearestVertex;
    ofVec3f nearestVertexCam;
    int nearestIndex = 0;
    int mouseX = ofGetMouseX();
    int mouseY = ofGetMouseY();
    ofVec2f mouse(mouseX, mouseY);
    for(int i = 0; i < n; i++) {
        ofVec3f cur = myCam.worldToScreen(mesh.getVertex(i));
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
    ofVec2f origXY = ofVec2f(ofMap(nearestVertexCam.x,0,fbo.getWidth(),0,sizeX),ofMap(nearestVertexCam.y,0,fbo.getHeight(),0,sizeY));
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
void ofxDVS::draw() {
    
    myCam.begin();
    ofTranslate(ofPoint(-ofGetWidth()/2,-ofGetHeight()/2));
    drawFrames();
    drawImageGenerator(); // if dvs.drawImageGen
    drawSpikes();         // if dvs.doDrawSpikes
    //drawImu6();
    myCam.end();
    
    drawMouseDistanceToSpikes();
}

//--------------------------------------------------------------
void ofxDVS::setDrawSpikes(bool doDraw){
    doDrawSpikes = doDraw;
}

//--------------------------------------------------------------
bool ofxDVS::getDrawSpikes(){
    return doDrawSpikes;
}

//--------------------------------------------------------------
void ofxDVS::setIMU(bool value) {
    thread.lock();
    thread.imuStatus = value;
    thread.unlock();
}

//--------------------------------------------------------------
void ofxDVS::setDVS(bool value) {
    thread.lock();
    thread.dvsStatus = value;
    thread.unlock();
}

//--------------------------------------------------------------
void ofxDVS::setAPS(bool value) {
    thread.lock();
    thread.apsStatus = value;
    thread.unlock();
}

//--------------------------------------------------------------
void ofxDVS::changeAps() {
    thread.lock();
    bool current_status = thread.apsStatus;
    if(current_status){
        thread.apsStatus = false;
        apsStatus = false;
        ofLog(OF_LOG_NOTICE,"Aps Disabled\n");
    }else{
        thread.apsStatus = true;
        apsStatus = true;
        ofLog(OF_LOG_NOTICE,"Aps Enabled\n");
    }
    thread.unlock();
}

//--------------------------------------------------------------
void ofxDVS::changeDvs() {
    thread.lock();
    bool current_status = thread.dvsStatus;
    if(current_status){
        thread.dvsStatus = false;
        dvsStatus = false;
        ofLog(OF_LOG_NOTICE,"Dvs Disabled\n");
    }else{
        thread.dvsStatus = true;
        dvsStatus = true;
        ofLog(OF_LOG_NOTICE,"Dvs Enabled\n");
    }
    thread.unlock();
}


//--------------------------------------------------------------
void ofxDVS::changeStats() {
    bool current_status = statsStatus;
    if(current_status){
        statsStatus = false;
        ofLog(OF_LOG_NOTICE,"Stats Disabled\n");
    }else{
        statsStatus = true;
        ofLog(OF_LOG_NOTICE,"Stats Enabled\n");
    }
}


//--------------------------------------------------------------
void ofxDVS::changeImu() {
    thread.lock();
    bool current_status = thread.imuStatus;
    if(current_status){
        thread.imuStatus = false;
        imuStatus = false;
        ofLog(OF_LOG_NOTICE,"Imu Disabled\n");
    }else{
        thread.imuStatus = true;
        imuStatus = true;
        ofLog(OF_LOG_NOTICE,"Imu Enabled\n");
    }
    thread.unlock();
}


//--------------------------------------------------------------
void ofxDVS::drawSpikes() {
    
    if(doDrawSpikes){
    
        for (int i = 0; i < packetsPolarity.size(); i++) {
            int x =(int)packetsPolarity[i].pos.x;
            int y =(int)packetsPolarity[i].pos.y;
            if(packetsPolarity[i].valid){
                visualizerMap[x][y] += 65;
            }
            //thread_alpha.visualizerMap[x][y] = visualizerMap[x][y];
        }
        for( int i=0; i<sizeX; ++i ) {
            for( int j=0; j<sizeY; ++j ) {
                if(visualizerMap[i][j] != 0){
                    visualizerMap[i][j] -= fsint;
                    if(visualizerMap[i][j] < 0){
                        visualizerMap[i][j] = 0;
                    }
                }
            }
        }

        
        mesh.clear();
        vector<polarity> packets = getPolarity();
        for(int i=0;i < packets.size();i++) {
            
            int x = (int)packetsPolarity[i].pos.x;
            int y = (int)packetsPolarity[i].pos.y;
            int alpha = 255;//(int)ceil(visualizerMap[x][y]);
            
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
            long timeus = 0;
            if(m == 0){
                timeus = 0;
            }else{
                timeus = tdiff>>m;
            }
            mesh.addVertex(ofVec3f(ofMap(packets[i].pos.x,0,sizeX,0,ofGetWidth()),ofMap(packets[i].pos.y,sizeY,0,0,ofGetHeight()), timeus));
            mesh.addTexCoord(ofVec2f(packets[i].pos.x,packets[i].pos.y));
            //int alphaus = (int)ceil(((float)tdiff/(float)nus)*256);
            if(packets[i].pol){
                mesh.addColor(ofColor(spkOnR[paletteSpike],spkOnG[paletteSpike],spkOnB[paletteSpike],alpha));
            }else{
                mesh.addColor(ofColor(spkOffR[paletteSpike],spkOffG[paletteSpike],spkOffB[paletteSpike],alpha));
            }
        }
        mesh.setMode(OF_PRIMITIVE_POINTS);
        ofPushMatrix();
        mesh.draw();
        ofPopMatrix();

        /*fbo.begin();
        ofClear(255,255,255, 0);
        //ofFill();
        //ofSetColor(ofNoise( ofGetFrameNum() ) * 255 * 5, 255);
        float scalex = (float) 1024;
        float scaley = (float) 768;
        float scaleFx,scaleFy;

        scaleFx = scalex/sizeX;
        scaleFy = scaley/sizeY;

        //thread_alpha.lock();
        //float max_alpha=0;
        for (int i = 0; i < packetsPolarity.size(); i++) {
            int x =(int)packetsPolarity[i].pos.x;
            int y =(int)packetsPolarity[i].pos.y;
            if(packetsPolarity[i].valid){
                visualizerMap[x][y] += 65;
            }
            //thread_alpha.visualizerMap[x][y] = visualizerMap[x][y];
        }
        for( int i=0; i<sizeX; ++i ) {
            for( int j=0; j<sizeY; ++j ) {
                if(visualizerMap[i][j] != 0){
                    visualizerMap[i][j] -= fsint;
                    if(visualizerMap[i][j] < 0){
                        visualizerMap[i][j] = 0;
                    }
                }
            }
        }
        //thread_alpha.unlock();
        
        for (int i = 0; i < packetsPolarity.size(); i++) {
            if(packetsPolarity[i].valid == false){
                continue;
            }
            ofPushStyle();
            //thread_alpha.lock();
            int x = (int)packetsPolarity[i].pos.x;
            int y = (int)packetsPolarity[i].pos.y;
            int alpha = (int)ceil(visualizerMap[x][y]);//thread_alpha.visualizerMap[(int)packetsPolarity[i].pos.x][(int)packetsPolarity[i].pos.y]*255;
            //thread_alpha.unlock();

            if(alpha > 255){
                alpha = 255;
            }
            //cout << alpha << endl;
            if(packetsPolarity[i].pol) {
                ofSetColor(spkOnR[paletteSpike],spkOnG[paletteSpike],spkOnB[paletteSpike],alpha);
            }
            else {
                ofSetColor(spkOffR[paletteSpike],spkOffG[paletteSpike],spkOffB[paletteSpike],alpha);
            }
            ofDrawCircle((int)(x), (int)(y), 1.0);
            ofPopStyle();
        }
        fbo.end();
        fbo.draw(0,0,ofGetWidth(),ofGetHeight());
        //packetsPolarity.clear();*/
    }
}

//--------------------------------------------------------------
void ofxDVS::drawFrames() {
    // draw last frame in packets
    for (int i = 0; i < packetsFrames.size(); i++) {
        packetsFrames[i].singleFrame.draw(0,0,ofGetWidth(),ofGetHeight());
    }
    //packetsFrames.clear();
}

//--------------------------
void ofxDVS::initBAfilter(){
    baFilterMap=new long*[sizeX];
    for( int i=0; i<sizeX; ++i ) {
        baFilterMap[i] = new long[sizeY];
        for( int j=0; j<sizeY; ++j ) {
            baFilterMap[i][j] = 0;
        }
    }
    
    BAdeltaT = 3000;
}

//--------------------------
void ofxDVS::initVisualizerMap(){
    visualizerMap=new float*[sizeX];
    for( int i=0; i<sizeX; ++i ) {
        visualizerMap[i] = new float[sizeY];
        for( int j=0; j<sizeY; ++j ) {
            visualizerMap[i][j] = 0.0;
        }
    }
}

//--------------------------
// Background Activity Filter
void ofxDVS::updateBAFilter(){
    
    for (int i = 0; i < packetsPolarity.size(); i++) {
        ofPoint pos = packetsPolarity[i].pos;

        // get last spike time
        int lastTS = baFilterMap[(int)pos.x][(int)pos.y];
        int ts = packetsPolarity[i].timestamp;
        
        if (( (ts - lastTS) >= BAdeltaT) || (lastTS == 0)) {
            // Filter out invalid, simply invalid them
            packetsPolarity[i].valid = false;
        }
        
        // Update neighboring region.
        size_t sizeMaxX = (sizeX - 1);
        size_t sizeMaxY = (sizeY - 1);

        int x = pos.x;
        int y = pos.y;
        
        if (x > 0) {
            baFilterMap[x - 1][y] = ts;
        }
        if (x < sizeMaxX) {
            baFilterMap[x + 1][y] = ts;
        }
        
        if (y > 0) {
            baFilterMap[x][y - 1] = ts;
        }
        if (y < sizeMaxY) {
            baFilterMap[x][y + 1] = ts;
        }
        
        if (x > 0 && y > 0) {
            baFilterMap[x - 1][y - 1] = ts;
        }
        if (x < sizeMaxX && y < sizeMaxY) {
            baFilterMap[x + 1][y + 1] = ts;
        }
        
        if (x > 0 && y < sizeMaxY) {
            baFilterMap[x - 1][y + 1] = ts;
        }
        if (x < sizeMaxX && y > 0) {
            baFilterMap[x + 1][y - 1] = ts;
        }
    
    }
}


//--------------------------------------------------------------
void ofxDVS::loadFile() {
    ofFileDialogResult result = ofSystemLoadDialog("Load aedat file");
    if(result.bSuccess) {
        path = result.getPath();
        // load your file at `path`
        changePath();
    }
}
    
//--------------------------------------------------------------
ofTexture* ofxDVS::getTextureRef() {
    return tex;
}

//--------------------------------------------------------------
void ofxDVS::exit() {
    
    // stop the thread
    thread.stopThread();
    
    if(isRecording){
        // close file
        myFile.close();
    }

}

const char * ofxDVS::chipIDToName(int16_t chipID, bool withEndSlash) {
    switch (chipID) {
        case 0:
            return ((withEndSlash) ? ("DAVIS240A/") : ("DAVIS240A"));
            break;
            
        case 1:
            return ((withEndSlash) ? ("DAVIS240B/") : ("DAVIS240B"));
            break;
            
        case 2:
            return ((withEndSlash) ? ("DAVIS240C/") : ("DAVIS240C"));
            break;
            
        case 3:
            return ((withEndSlash) ? ("DAVIS128/") : ("DAVIS128"));
            break;
            
        case 4:
            return ((withEndSlash) ? ("DAVIS346A/") : ("DAVIS346A"));
            break;
            
        case 5:
            return ((withEndSlash) ? ("DAVIS346B/") : ("DAVIS346B"));
            break;
            
        case 6:
            return ((withEndSlash) ? ("DAVIS640/") : ("DAVIS640"));
            break;
            
        case 7:
            return ((withEndSlash) ? ("DAVISHet640/") : ("DAVISHet640"));
            break;
            
        case 8:
            return ((withEndSlash) ? ("DAVIS208/") : ("DAVIS208"));
            break;
            
        case 9:
            return ((withEndSlash) ? ("DAVIS346Cbsi/") : ("DAVIS346Cbsi"));
            break;
    }
    
    return ((withEndSlash) ? ("Unknown/") : ("Unknown"));
}

// --- Sort packet container by timestamp and type ID
int ofxDVS::packetsFirstTimestampThenTypeCmp(const void *a, const void *b) {
    const caerEventPacketHeader *aa = (const caerEventPacketHeader *)a;
    const caerEventPacketHeader *bb = (const caerEventPacketHeader *)b;
    
    if(*aa == NULL && *bb == NULL){
        return(0);
    }else if( *aa == NULL && *bb != NULL){
        return(-1);
    }else if( *aa != NULL && *bb == NULL){
        return(1);
    }
    
    // Sort first by timestamp of the first event.
    int32_t eventTimestampA = caerGenericEventGetTimestamp(caerGenericEventGetEvent(*aa, 0), *aa);
    int32_t eventTimestampB = caerGenericEventGetTimestamp(caerGenericEventGetEvent(*bb, 0), *bb);
    
    if (eventTimestampA < eventTimestampB) {
        return (-1);
    }
    else if (eventTimestampA > eventTimestampB) {
        return (1);
    }
    else {
        // If equal, further sort by type ID.
        int16_t eventTypeA = caerEventPacketHeaderGetEventType(*aa);
        int16_t eventTypeB = caerEventPacketHeaderGetEventType(*bb);
        
        if (eventTypeA < eventTypeB) {
            return (-1);
        }
        else if (eventTypeA > eventTypeB) {
            return (1);
        }
        else {
            return (0);
        }
    }
}

string ofxDVS::getUserHomeDir()
{
    std::string homeDir = "";
    
#ifdef _WIN32
    char szPath[ MAX_PATH ];
    
    if ( S_OK == SHGetFolderPathA( NULL, CSIDL_PROFILE, NULL, 0, szPath ) )
    {
        homeDir = path;
    }
#elif defined(__unix__) || defined(__APPLE__)
    uid_t uid = getuid();
    struct passwd* pwd = getpwuid( uid );
    
    if ( NULL != pwd )
    {
        homeDir = pwd->pw_dir;
    }
#endif
    
    return homeDir;
}

// Image Generator
//--------------------------------------------------------------
void ofxDVS::initImageGenerator(){
    spikeFeatures = new float*[sizeX];
    for(int i = 0; i < sizeX; ++i){
        spikeFeatures[i] = new float[sizeY];
    }
    for(int i = 0; i < sizeX; ++i){
        for(int j = 0; j < sizeY; ++j){
            spikeFeatures[i][j] = 0.0;
        }
    }
    imageGenerator.allocate(sizeX, sizeY, OF_IMAGE_COLOR_ALPHA);
    rectifyPolarities = false;
    numSpikes = 2000;
    counterSpikes = 0;
    drawImageGen = false;
}

//--------------------------------------------------------------
void ofxDVS::drawImageGenerator() {
    
    // draw last imagegenerator frame
    //ofEnableAlphaBlending();
    //ofDisableAlphaBlending();
    if(drawImageGen){
        imageGenerator.draw(0,0,ofGetWidth(),ofGetHeight());
    }
}

//--------------------------------------------------------------
void ofxDVS::drawImu6() {
    
    //fbo.begin();
    ofPushStyle();
    //ofSetColor(0,0,0);
    ofVec3f arrowTailPoint = ofVec3f(100,100,0);
    //ofVec3f arrowHeadPoint = ofVec3f(150,100,0);
    //ofDrawArrow(arrowTailPoint, arrowHeadPoint, 20.0);

    for(size_t i=0; i < packetsImu6.size(); i++){
        cout << "x :" << packetsImu6[i].gyro.x << " y :" << packetsImu6[i].gyro.y <<
        " z :" << packetsImu6[i].gyro.z << endl;
         ofSetColor(0,0,0);
       ofDrawArrow(arrowTailPoint,packetsImu6[i].gyro, 10.0);
         ofSetColor(255,255,0);
       ofDrawArrow(arrowTailPoint,packetsImu6[i].accel, 10.0);
    }
    ofPopStyle();
    //fbo.end();
    //fbo.draw(0,0,ofGetWidth(),ofGetHeight());
    //packetsImu6.clear();
}


//--------------------------------------------------------------
void ofxDVS::setDrawImageGen(bool doDraw){
    drawImageGen = doDraw;
}

//--------------------------------------------------------------
bool ofxDVS::getDrawImageGen(){
    return drawImageGen;
}

//--------------------------------------------------------------
void ofxDVS::setImageAccumulatorSpikes(int value){
    numSpikes = value;
}

//--------------------------------------------------------------
void ofxDVS::set3DTime(int i){
    if(i == 0){
        m = 0;
    }else if(i == 1){
        m = 8;
    }else if(i == 2){
        m = 6;
    }else if(i == 3){
        m = 4;
    }else if(i == 4){
        m = 2;
    }
}

//--------------------------------------------------------------
void ofxDVS::updateImageGenerator(){
    
    for (int i = 0; i < packetsPolarity.size(); i++) {
        // only valid
        if(packetsPolarity[i].valid){
            ofPoint pos = packetsPolarity[i].pos;
            
            // update surfaceMap
            if(packetsPolarity[i].pol){
                spikeFeatures[(int)pos.x][(int)pos.y] += 1.0;
            }else{
                spikeFeatures[(int)pos.x][(int)pos.y] -= 1.0;
            }
            counterSpikes = counterSpikes+1;
        }
    }
    
    if(numSpikes <= counterSpikes){

        counterSpikes = 0;
        //ofLog(OF_LOG_WARNING,"Generate Image \n");
        // normalize
        int sum = 0, count = 0;
        for (int i = 0; i < sizeX; i++) {
            for (int j = 0; j < sizeY; j++) {
                if (spikeFeatures[i][j] != 0.0) {
                    sum += spikeFeatures[i][j];
                    count++;
                }
            }
        }
        float mean = sum / count;
        float var = 0;
        for (int i = 0; i < sizeX; i++) {
            for (int j = 0; j < sizeY; j++) {
                if (spikeFeatures[i][j] != 0.0) {
                    float f = spikeFeatures[i][j] - mean;
                    var += f * f;
                }
            }
        }
        
        float sig = sqrt(var / count);
        if (sig < (0.1f / 255.0f)) {
            sig = 0.1f / 255.0f;
        }
        
        float numSDevs = 3;
        float mean_png_gray, range, halfrange;
        
        if (rectifyPolarities) {
            mean_png_gray = 0; // rectified
        }
        
        if (rectifyPolarities) {
            range = numSDevs * sig * (1.0f / 256.0f); //256 included here for nullhop reshift
            halfrange = 0;
        }
        
        for (int col_idx = 0; col_idx < sizeX; col_idx++) {
            for (int row_idx = 0; row_idx < sizeY; row_idx++) {
                ofColor this_pixel;
                this_pixel.set(0,0,0,1);
                imageGenerator.setColor(col_idx,row_idx,this_pixel);
            }
        }
        for (int col_idx = 0; col_idx < sizeX; col_idx++) {
            for (int row_idx = 0; row_idx < sizeY; row_idx++) {
                if (spikeFeatures[col_idx][row_idx] == 0) {
                    spikeFeatures[col_idx][row_idx] = mean_png_gray;
                }
                else {
                    float f = (spikeFeatures[col_idx][row_idx] + halfrange) / range;
                    if (f > 255) {
                        f = 255; //255 included here for nullhop reshift
                    }else if (f < 0) {
                        f = 0;
                    }
                    ofColor this_pixel;
                    //spkOnR[paletteSpike],spkOnG[paletteSpike],spkOnB[paletteSpike]
                    this_pixel.set((int)floor(f), spkOnR[paletteSpike],spkOnG[paletteSpike], 125);
                    imageGenerator.setColor(col_idx,row_idx,this_pixel);
                }
            }
        }
        // clear map
        for (int col_idx = 0; col_idx < sizeX; col_idx++) {
            for (int row_idx = 0; row_idx < sizeY; row_idx++) {
                spikeFeatures[col_idx][row_idx]  = 0.0;
            }
        }
        imageGenerator.update();
    }
}

//--------------------------------------------------------------
void ofxDVS::changeTargetSpeed(float val){
    targetSpeed = targetSpeed + val;
    ofLog(OF_LOG_NOTICE, "Target speed is now %f", targetSpeed);
}

//--------------------------------------------------------------
void ofxDVS::setTargetSpeed(float val){
	targetSpeed = val;
}

//--------------------------------------------------------------
float ofxDVS::getTargetSpeed(){
    return(targetSpeed);
}

//--------------------------------------------------------------
void ofxDVS::changePause(){
    if(paused){
        paused = false;
        thread.lock();
        thread.paused = paused;
        thread.unlock();
    }else{
        paused = true;
        thread.lock();
        thread.paused = paused;
        thread.unlock();
    }
}

//--------------------------------------------------------------
void ofxDVS::setPause(bool value){
	paused = value;
	thread.lock();
	thread.paused = paused;
	thread.unlock();
}

//--------------------------------------------------------------
ofImage ofxDVS:: getImageGenerator(){
    
    return imageGenerator;
    
}


