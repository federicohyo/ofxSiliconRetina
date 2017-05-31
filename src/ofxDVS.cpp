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
    
}

//--------------------------------------------------------------
void ofxDVS::setup() {
        
    thread.startThread();   // start usb thread

    // get camera size after ready
    LOCK_CHECK:
    thread.lock();
    if(thread.deviceReady!=true){
    	thread.unlock();
    	goto LOCK_CHECK;
    }
	sizeX = thread.sizeX;
	sizeY = thread.sizeY;
	thread.unlock();

	// init framebuffer
    fbo.allocate(sizeX, sizeY, GL_RGBA32F);
    tex = &fbo.getTexture();

    // render spike colored
    initSpikeColors();

    // start the thread
    apsStatus = true;       // enable aps
    dvsStatus = true;       // enable dvs
    imuStatus = true;       // enable imu
    maxContainerQueued = 100; // at most accumulates 100 packaetcontainers before dropping
    packetContainer = NULL;

    //init spikefeature/imagegenerator
    spikeFeatures = new float*[sizeX];
    for(int i = 0; i < sizeX; ++i){
        spikeFeatures[i] = new float[sizeY];
    }
    for(int i = 0; i < sizeX; ++i){
        for(int j = 0; j < sizeY; ++j){
            spikeFeatures[i][j] = 0.0;
        }
    }
    imageGenerator.allocate(sizeX, sizeY, OF_IMAGE_COLOR);
    rectifyPolarities = true;
    numSpikes = 2000;
    counterSpikes = 0;
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
    spkOnG[2] = 255;
    spkOnB[2] = 255;
    spkOffR[2] = 255;
    spkOffG[2] = 255;
    spkOffB[2] = 0;
    
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
bool ofxDVS::organizeData(caerEventPacketContainer packetContainer){

    if (packetContainer == NULL) {
        return(false); // Skip if nothing there.
    }
    
    
    int32_t packetNum = caerEventPacketContainerGetEventPacketsNumber(packetContainer);
    
    
    for (int32_t i = 0; i < packetNum; i++) {
        
        polarity nuPack;
        frame nuPackFrames;
        imu6 nuPackImu6;
        
        caerEventPacketHeader packetHeader = caerEventPacketContainerGetEventPacket(packetContainer, i);
        if (packetHeader == NULL) {
            //ofLog(OF_LOG_WARNING,"Packet %d is empty (not present).\n", i);
            continue; // Skip if nothing there.
        }
        
        //ofLog(OF_LOG_WARNING,"Packet %d of type %d -> size is %d.\n", i, caerEventPacketHeaderGetEventType(packetHeader),
        //           caerEventPacketHeaderGetEventNumber(packetHeader));
        
        if (i == IMU6_EVENT) {
            
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
            
            packetsImu6.push_back(nuPackImu6);
            CAER_IMU6_ITERATOR_VALID_END
        }
        if (i == POLARITY_EVENT) {
            
            packetsPolarity.clear();
            packetsPolarity.shrink_to_fit();
            
            caerPolarityEventPacket polarity = (caerPolarityEventPacket) packetHeader;
            
            CAER_POLARITY_ITERATOR_VALID_START(polarity)
            nuPack.timestamp = caerPolarityEventGetTimestamp64(caerPolarityIteratorElement, polarity);
            
            nuPack.pos.x = caerPolarityEventGetX(caerPolarityIteratorElement);
            nuPack.pos.y = caerPolarityEventGetY(caerPolarityIteratorElement);
            
            nuPack.pol = caerPolarityEventGetPolarity(caerPolarityIteratorElement);
            
            packetsPolarity.push_back(nuPack);
            
            CAER_POLARITY_ITERATOR_VALID_END
        }
        if (i == FRAME_EVENT){
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
            
            packetsFrames.push_back(nuPackFrames);
            
            CAER_FRAME_ITERATOR_VALID_END
        }
        
    }
    
    return(true);

}

//--------------------------------------------------------------
void ofxDVS::update() {
    
    // Copy data from usbThread
    thread.lock();
    for(int i=0; i<thread.container.size(); i++){
        packetContainer = thread.container.back(); // apparently this is a pointer
        thread.container.pop_back();
        organizeData(packetContainer);
        caerEventPacketContainerFree(packetContainer);  // free all packet containers here
    }
    // done with the resource
    thread.unlock();
    
    // check how fast we are going, if we are too slow, drop some data
    thread.lock();
    if(thread.container.size() > maxContainerQueued){
        ofLog(OF_LOG_WARNING, "Visualization is too slow, dropping events to keep real-time.");
        thread.container.clear();
    }
    thread.unlock();

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
void ofxDVS::draw() {
    drawSpikes();
}

//--------------------------------------------------------------
void ofxDVS::changeAps() {
    thread.lock();
    bool current_status = thread.apsStatus;
    if(current_status){
        thread.apsStatus = false;
        ofLog(OF_LOG_NOTICE,"Aps Disabled\n");
    }else{
        thread.apsStatus = true;
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
        ofLog(OF_LOG_NOTICE,"Dvs Disabled\n");
    }else{
        thread.dvsStatus = true;
        ofLog(OF_LOG_NOTICE,"Dvs Enabled\n");
    }
    thread.unlock();
}

//--------------------------------------------------------------
void ofxDVS::changeImu() {
    thread.lock();
    bool current_status = thread.imuStatus;
    if(current_status){
        thread.imuStatus = false;
        ofLog(OF_LOG_NOTICE,"Imu Disabled\n");
    }else{
        thread.imuStatus = true;
        ofLog(OF_LOG_NOTICE,"Imu Enabled\n");
    }
    thread.unlock();
}


//--------------------------------------------------------------
void ofxDVS::drawSpikes() {
    //fbo.begin();
    //ofClear(0,255);
    //ofFill();
    //ofSetColor(ofNoise( ofGetFrameNum() ) * 255 * 5, 255);

    float scalex = (float) ofGetWidth();
    float scaley = (float) ofGetHeight();
    float scaleFx,scaleFy;
    scaleFx = scalex/sizeX;
    scaleFy = scaley/sizeY;
    for (int i = 0; i < packetsPolarity.size(); i++) {
        ofPushStyle();
        if(packetsPolarity[i].pol) {
            ofSetColor(spkOnR[paletteSpike],spkOnG[paletteSpike],spkOnB[paletteSpike],spkOnA);
        }
        else {
            ofSetColor(spkOffR[paletteSpike],spkOffG[paletteSpike],spkOffB[paletteSpike],spkOffA);
        }
        ofDrawCircle((int) packetsPolarity[i].pos.x*scaleFx, (int)packetsPolarity[i].pos.y*scaleFy, 1);
        ofPopStyle();
    }

    //fbo.end();
    //fbo.draw(0,0,ofGetWidth(),ofGetHeight());
}

//--------------------------------------------------------------
void ofxDVS::drawFrames() {

    // draw last frame in packets
    for (int i = 0; i < packetsFrames.size(); i++) {
        packetsFrames[i].singleFrame.draw(0,0,ofGetWidth(),ofGetHeight());
    }

}

//--------------------------------------------------------------
void ofxDVS::drawImageGenerator() {

    // draw last imagegenerator frame
    imageGenerator.draw(0,0,ofGetWidth(),ofGetHeight());
    //for (int col_idx = 0; col_idx < sizeX; col_idx++) {
    //    for (int row_idx = 0; row_idx < sizeY; row_idx++) {
    //        ofColor thispix= imageGenerator.getColor(col_idx, row_idx);
   //         ofLog(OF_LOG_WARNING,"Color Pixel red %d  \n", thispix.r);
    //    }
   // }

}


//--------------------------------------------------------------
void ofxDVS::updateImageGenerator(){
    
    for (int i = 0; i < packetsPolarity.size(); i++) {
        ofPoint pos = packetsPolarity[i].pos;
        // update surfaceMap
        if(packetsPolarity[i].pol){
            spikeFeatures[(int)pos.x][(int)pos.y] += 1.0;
        }else{
            spikeFeatures[(int)pos.x][(int)pos.y] += 1.0;
        }
        counterSpikes = counterSpikes+1;
    }

    if(numSpikes <= counterSpikes){

        counterSpikes = 0;
        ofLog(OF_LOG_WARNING,"Generate Image \n");
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
        
        //ofLog(OF_LOG_WARNING,"var %f \n", var);
        //ofLog(OF_LOG_WARNING,"mean %f \n", mean);
        //ofLog(OF_LOG_WARNING,"range %f \n", range);
        
        for (int col_idx = 0; col_idx < sizeX; col_idx++) {
            for (int row_idx = 0; row_idx < sizeY; row_idx++) {
                ofColor this_pixel;
                this_pixel.set(0,0,0);
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
                    this_pixel.set((int)floor(f),(int)floor(f),(int)floor(f));
                    imageGenerator.setColor(col_idx,row_idx,ofColor(this_pixel.r, this_pixel.g, this_pixel.b));
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
ofImage ofxDVS:: getImageGenerator(){
    
    return imageGenerator;
    
}

//--------------------------------------------------------------
void ofxDVS::drawImu6() {

}


//--------------------------------------------------------------
ofTexture* ofxDVS::getTextureRef() {
    return tex;
}

//--------------------------------------------------------------
void ofxDVS::exit() {
    // stop the thread
    thread.stopThread();
}
