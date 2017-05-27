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


class usbThread: public ofThread
{
public:
 
    void threadedFunction()
    {
        
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
        
        // Turn on Autoexposure if device has APS
#if defined(DAVIS346) || defined(DAVIS240)
        caerDeviceConfigSet(camera_handle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_AUTOEXPOSURE, true);
#endif
        
        // Now let's get start getting some data from the device. We just loop, no notification needed.
        caerDeviceDataStart(camera_handle, NULL, NULL, NULL, NULL, NULL);

        // Let's turn on blocking data-get mode to avoid wasting resources.
        caerDeviceConfigSet(camera_handle, CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);
        
        // get current frame status
        apsStatus = true;
        apsStatusLocal = true;
        dvsStatus = true;
        dvsStatusLocal = true;
        imuStatus = true;
        imuStatusLocal = true;
        
        while(isThreadRunning())
        {
            lock();
            packetContainerT = NULL;
            packetContainerT = caerDeviceDataGet(camera_handle);
            if (packetContainerT != NULL){
                container.push_back(packetContainerT);
            }
            unlock();

            //check aps status
            if( apsStatus != apsStatusLocal){
                apsStatusLocal = apsStatus;
                //enable disable frames
#if defined(DAVIS346) || defined(DAVIS240)
                caerDeviceConfigSet(camera_handle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, apsStatusLocal);
#endif
            }
            //check dvs status
            if( dvsStatus != dvsStatusLocal){
                dvsStatusLocal = dvsStatus;
                //enable disable frames
#if defined(DAVIS346) || defined(DAVIS240)
                caerDeviceConfigSet(camera_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_RUN, dvsStatusLocal);
#elif defined(DVS128)
                caerDeviceConfigSet(camera_handle, DVS128_CONFIG_DVS, DVS128_CONFIG_DVS_RUN, dvsStatusLocal);
#endif
            }
            //check imu status
            if( imuStatus != imuStatusLocal){
                imuStatusLocal = imuStatus;
                //enable disable frames
#if defined(DAVIS346) || defined(DAVIS240)
                caerDeviceConfigSet(camera_handle, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN, imuStatusLocal);
#endif
            }

        }
    }
    
    caerDeviceHandle camera_handle;
    vector<caerEventPacketContainer> container;
    caerEventPacketContainer packetContainerT;
    
    // enable disable aps / dvs /imu
    bool apsStatus;
    bool apsStatusLocal;
    bool dvsStatus;
    bool dvsStatusLocal;
    bool imuStatus;
    bool imuStatusLocal;
    
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
    void loopColor();
    void exit();
    bool organizeData(caerEventPacketContainer packetContainer);
    void changeAps(); // enable / disable aps
    void changeDvs(); // enable / disable dvs
    void changeImu(); // enable / disable imu

    // Camera
    std::atomic_bool globalShutdown = ATOMIC_VAR_INIT(false);
    void globalShutdownSignalHandler(int signal);
    
    // Textures and framebuffer
    ofFbo fbo;
    ofTexture* tex;
    ofTexture* getTextureRef();
    
    // Data containers
    vector<polarity> packetsPolarity;
    vector<frame> packetsFrames;
    vector<imu6> packetsImu6;
    caerEventPacketContainer packetContainer;
    
    // Data functions
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
    int maxContainerQueued;
    
    // thread usb
    usbThread thread;
    bool apsStatus; // enable/disable aps
    bool dvsStatus; // enable/disable dvs
    bool imuStatus; // enable/disable imu
};


#endif /* ofxDVS_hpp */
