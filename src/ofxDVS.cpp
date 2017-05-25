//
//  ofxDVS.cpp
//  ofxDVS
//
//  Created by Federico Corradi on 19.05.17.
//
//

#include "ofxDVS.hpp"

//--------------------------------------------------------------
ofxDVS::ofxDVS() {
    
    
}

//--------------------------------------------------------------
void ofxDVS::setup() {
    // start the camera
#ifdef DAVIS346
    // Open a DAVIS, give it a device ID of 1, and don't care about USB bus or SN restrictions.
    camera_handle = caerDeviceOpen(1, CAER_DEVICE_DAVIS_FX3, 0, 0, NULL);
#endif
#ifdef DAVIS240
    // Open a DAVIS, give it a device ID of 1, and don't care about USB bus or SN restrictions.
    camera_handle = caerDeviceOpen(1, CAER_DEVICE_DAVIS_FX2, 0, 0, NULL);
#endif
#ifdef DVS128
    // Open a DVS128, give it a device ID of 1, and don't care about USB bus or SN restrictions.
    camera_handle = caerDeviceOpen(1, CAER_DEVICE_DVS128, 0, 0, NULL);
#endif
    
    if (camera_handle == NULL) {
        printf("error opening the device\n");
        return (EXIT_FAILURE);
    }
    
    // Send the default configuration before using the device.
    // No configuration is sent automatically!
    caerDeviceSendDefaultConfig(camera_handle);
    
    // Now let's get start getting some data from the device. We just loop, no notification needed.
    caerDeviceDataStart(camera_handle, NULL, NULL, NULL, NULL, NULL);
    
    fbo.allocate(SIZEX*MUL, SIZEY*MUL, GL_RGBA32F);
    outImg.allocate(SIZEX*MUL, SIZEY*MUL, OF_IMAGE_COLOR);
    tex = &fbo.getTexture();
    
    
}


//--------------------------------------------------------------
void ofxDVS::update() {
    
    //sensor
    caerEventPacketContainer packetContainer = caerDeviceDataGet(camera_handle);
    if (packetContainer == NULL) {
        return; // Skip if nothing there.
    }
    
    int32_t packetNum = caerEventPacketContainerGetEventPacketsNumber(packetContainer);
    
    if(DEBUG){
        printf("\nGot event container with %d packets (allocated).\n", packetNum);
    }
    
    packets.clear();
    
    for (int32_t i = 0; i < packetNum; i++) {
        
        polarity nuPack;
        
        caerEventPacketHeader packetHeader = caerEventPacketContainerGetEventPacket(packetContainer, i);
        if (packetHeader == NULL) {
            if(DEBUG){
                printf("Packet %d is empty (not present).\n", i);
            }
            continue; // Skip if nothing there.
        }
        
        if(DEBUG){
            printf("Packet %d of type %d -> size is %d.\n", i, caerEventPacketHeaderGetEventType(packetHeader),
                   caerEventPacketHeaderGetEventNumber(packetHeader));
        }
        
        caerPolarityEventPacket polarity = (caerPolarityEventPacket) packetHeader;
        
        if (i == POLARITY_EVENT) {
            
            CAER_POLARITY_ITERATOR_VALID_START(polarity)
            nuPack.timestamp = caerPolarityEventGetTimestamp64(caerPolarityIteratorElement, polarity);
            
            nuPack.pos.x = caerPolarityEventGetX(caerPolarityIteratorElement);
            nuPack.pos.y = caerPolarityEventGetY(caerPolarityIteratorElement);
            
            nuPack.pol = caerPolarityEventGetPolarity(caerPolarityIteratorElement);
            
            packets.push_back(nuPack);
            
            CAER_POLARITY_ITERATOR_VALID_END
            
        }
        
    }
    
    caerEventPacketContainerFree(packetContainer);
    
    
}


//--------------------------------------------------------------
void ofxDVS::draw() {
    fbo.begin();
    ofClear(0,255);
    ofFill();
    //ofSetColor(ofNoise( ofGetFrameNum() ) * 255 * 5, 255);
    
    for (int i = 0; i < packets.size(); i++) {
        ofPushStyle();
        if(packets[i].pol) {
            ofSetColor(255, 255, 255, 255);
        }
        else {
            ofSetColor(0, 0, 0, 255);
        }
        ofPopStyle();
        ofDrawCircle(packets[i].pos.x*MUL, packets[i].pos.y*MUL, 1);
    }
    fbo.end();
    
    
    fbo.draw(0,0,ofGetWidth(),ofGetHeight());
    //   outImg.draw(0,0,ofGetWidth(),ofGetHeight());
}

//--------------------------------------------------------------
ofTexture* ofxDVS::getTextureRef() {
    return tex;
}

//--------------------------------------------------------------

//--------------------------------------------------------------
