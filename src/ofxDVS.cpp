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
    
    // Let's turn on blocking data-get mode to avoid wasting resources.
    caerDeviceConfigSet(camera_handle, CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);
    
    // Now let's get start getting some data from the device. We just loop, no notification needed.
    caerDeviceDataStart(camera_handle, NULL, NULL, NULL, NULL, NULL);
    
    fbo.allocate(SIZEX, SIZEY, GL_RGBA32F);
    outImg.allocate(SIZEX, SIZEY, OF_IMAGE_COLOR);
    tex = &fbo.getTexture();
    
    
}


//--------------------------------------------------------------
vector<polarity> ofxDVS::getPackets() {
    return packets;
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
    packetsFrames.clear();
    
    for (int32_t i = 0; i < packetNum; i++) {
        
        polarity nuPack;
        frame nuPackFrames;
        
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
        
        if (i == POLARITY_EVENT) {
            caerPolarityEventPacket polarity = (caerPolarityEventPacket) packetHeader;
            
            CAER_POLARITY_ITERATOR_VALID_START(polarity)
            nuPack.timestamp = caerPolarityEventGetTimestamp64(caerPolarityIteratorElement, polarity);
            
            nuPack.pos.x = caerPolarityEventGetX(caerPolarityIteratorElement);
            nuPack.pos.y = caerPolarityEventGetY(caerPolarityIteratorElement);
            
            nuPack.pol = caerPolarityEventGetPolarity(caerPolarityIteratorElement);
            
            packets.push_back(nuPack);
            
            CAER_POLARITY_ITERATOR_VALID_END
            
        }
        if (i == FRAME_EVENT){
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
            
            
            for (int32_t y = 0; y < nuPackFrames.lenghtY; y++) {
                for (int32_t x = 0; x < nuPackFrames.lenghtX; x++) {
                    
                    switch (nuPackFrames.frameChannels) {
                    
                        case GRAYSCALE: {
                            nuPackFrames.pixels[x][y] = U8T(caerFrameEventGetPixelUnsafe(caerFrameIteratorElement, x, y) >> 8);
                            break;
                        }
                        case RGB: {
                            nuPackFrames.pixelsR[x][y] = U8T(caerFrameEventGetPixelForChannelUnsafe(caerFrameIteratorElement, x, y, 0) >> 8);
                            nuPackFrames.pixelsG[x][y] = U8T(caerFrameEventGetPixelForChannelUnsafe(caerFrameIteratorElement, x, y, 1) >> 8);
                            nuPackFrames.pixelsB[x][y] = U8T(caerFrameEventGetPixelForChannelUnsafe(caerFrameIteratorElement, x, y, 2) >> 8);
                            break;
                        }
                        case RGBA:
                        default: {
                            nuPackFrames.pixelsR[x][y] = U8T(caerFrameEventGetPixelForChannelUnsafe(caerFrameIteratorElement, x, y, 0) >> 8);
                            nuPackFrames.pixelsG[x][y] = U8T(caerFrameEventGetPixelForChannelUnsafe(caerFrameIteratorElement, x, y, 1) >> 8);
                            nuPackFrames.pixelsB[x][y] = U8T(caerFrameEventGetPixelForChannelUnsafe(caerFrameIteratorElement, x, y, 2) >> 8);
                            nuPackFrames.pixelsA[x][y] = U8T(caerFrameEventGetPixelForChannelUnsafe(caerFrameIteratorElement, x, y, 3) >> 8);
                            break;
                        }
                            
                    }
                    
                }
            }

            packetsFrames.push_back(nuPackFrames);

            CAER_FRAME_ITERATOR_VALID_END

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
        ofDrawCircle(packets[i].pos.x, packets[i].pos.y, 1);
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
