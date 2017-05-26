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
    int info;
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
    
    /// Pixel array
    /// The pixel array is laid out row by row (increasing X axis), going
    /// from top to bottom (increasing Y axis).
    ofImage singleFrame;
    
};

struct imu6 {
    int info;
    int timestamp;
    ofVec3f accel;
    ofVec3f gyro;
};

class ofxDVS {
public:
    ofxDVS();
    
    // Methods
    void setup();
    void update();
    void draw();
    void drawSpikes();
    void drawFrames();
    void drawImu6();
    void initSpikeColors();
    
    // Camera
    std::atomic_bool globalShutdown = ATOMIC_VAR_INIT(false);
    void globalShutdownSignalHandler(int signal);
    caerDeviceHandle camera_handle;
    
    // Textures and framebuffer
    ofFbo fbo;
    ofTexture* tex;
    
    // Data containers
    vector<polarity> packetsPolarity;
    vector<frame> packetsFrames;
    vector<imu6> packetsImu6;
    
    // Data functions
    ofTexture* getTextureRef();
    vector<polarity> getPolarity();
    vector<frame> getFrames();
    
    // color palette for spikes
    int spkOnR[3];
    int spkOnG[3];
    int spkOnB[3];
    int spkOnA;
    int spkOffR[3];
    int spkOffG[3];
    int spkOffB[3];
    int spkOffA;
    int paletteSpike;
    
};

#endif /* ofxDVS_hpp */
