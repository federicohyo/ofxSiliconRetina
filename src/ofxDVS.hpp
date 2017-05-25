//
//  ofxDVS.hpp
//  ofxDVS
//
//  Created by Federico Corradi on 19.05.17.
//
//

#ifndef ofxDVS_hpp
#define ofxDVS_hpp

#include "libcaer.h"
#include "devices/davis.h"
#include "devices/dvs128.h"
#include <atomic>

/// PLEASE SELECT SENSOR DAVIS240 or DVS128
#define DAVIS240
// and decide on parameters
#define MUL 1
#define DEBUG 0

#ifdef DAVIS346
#define SIZEX 346
#define SIZEY 280
#endif

#ifdef DAVIS240
#define SIZEX 240
#define SIZEY 180
#endif

#ifdef DVS128
#define SIZEX 128
#define SIZEY 128
#endif

#include "ofMain.h"

struct polarity {
    ofPoint pos;
    int timestamp;
    bool pol;
};

class ofxDVS {
public:
    ofxDVS();
    
    void setup();
    void update();
    void draw();
    
    ofImage outImg;
    ofFbo fbo;
    ofTexture* tex;
    
    ofImage* getImage();
    ofTexture* getTextureRef();
    
    // Camera
    std::atomic_bool globalShutdown = ATOMIC_VAR_INIT(false);
    void globalShutdownSignalHandler(int signal);
    caerDeviceHandle camera_handle;
    
    vector<polarity> packets;
    
};

#endif /* ofxDVS_hpp */
