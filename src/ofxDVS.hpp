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

struct frame {
    /// Event information (ROI region, color channels, color filter). First because of valid mark.
    int info;
    /// Start of Frame (SOF) timestamp.
    int frameStart;
    /// End of Frame (EOF) timestamp.
    int frameEnd;
    /// Start of Exposure (SOE) timestamp.
    int exposureStart;
    /// End of Exposure (EOE) timestamp.
    int exposureEnd;
    /// X axis length in pixels.
    int lenghtX;
    /// Y axis length in pixels.
    int lenghtY;
    /// X axis position (upper left offset) in pixels.
    int positionX;
    /// Y axis position (upper left offset) in pixels.
    int positionY;
    
    enum caer_frame_event_color_channels frameChannels;
    
    /// Pixel array, 16 bit unsigned integers, normalized to 16 bit depth.
    /// The pixel array is laid out row by row (increasing X axis), going
    /// from top to bottom (increasing Y axis).
    int pixels[SIZEX][SIZEY]; // size 1 here for C++ compatibility.
    int pixelsR[SIZEX][SIZEY];
    int pixelsG[SIZEX][SIZEY];
    int pixelsB[SIZEX][SIZEY];
    int pixelsA[SIZEX][SIZEY];
    
    //ofPixels ofxpixels;
    
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
    vector<polarity> getPackets();
    
    // Camera
    std::atomic_bool globalShutdown = ATOMIC_VAR_INIT(false);
    void globalShutdownSignalHandler(int signal);
    caerDeviceHandle camera_handle;
    
    vector<polarity> packets;
    vector<frame> packetsFrames;
    
};

#endif /* ofxDVS_hpp */
