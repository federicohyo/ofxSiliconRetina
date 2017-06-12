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

#ifdef _WIN32
#include <windows.h>
#include <shlobj.h>
#elif defined(__unix__) || defined(__APPLE__)
#include <pwd.h>
#include <unistd.h>
#endif

#include <iostream>
#include <dirent.h>
#include <string.h>
#include <cstdio>

#include <string>

/// PLEASE SELECT SENSOR DAVIS or DVS128
#define DAVIS  1
#define DVS128 0

// and decide on parameters
#define DEBUG 0

#include "ofMain.h"

struct polarity {
    int info;
    ofPoint pos;
    int timestamp;
    bool pol;
    bool valid;
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
    
    bool valid;
    
};

struct imu6 {
    int info;
    int timestamp;
    ofVec3f accel;
    ofVec3f gyro;
    bool valid;
};


class usbThread: public ofThread
{
public:
 
    void parseSourceString(char *sourceString) {
        // Create SourceInfo node.

        // Determine sizes via known chip information.
        if (caerStrEquals(sourceString, "DVS128")) {
            sizeX = sizeY = 128;
        }
        else if (caerStrEquals(sourceString, "DAVIS240A") || caerStrEquals(sourceString, "DAVIS240B")
                 || caerStrEquals(sourceString, "DAVIS240C")) {
            sizeX = 240;
            sizeY = 180;
        }
        else if (caerStrEquals(sourceString, "DAVIS128")) {
            sizeX = sizeY = 128;
        }
        else if (caerStrEquals(sourceString, "DAVIS346A") || caerStrEquals(sourceString, "DAVIS346B")
                 || caerStrEquals(sourceString, "DAVIS346Cbsi")) {
            sizeX = 346;
            sizeY = 260;
        }
        else if (caerStrEquals(sourceString, "DAVIS640")) {
            sizeX = 640;
            sizeY = 480;
        }
        else if (caerStrEquals(sourceString, "DAVISHet640")) {
            sizeX = 640;
            sizeY = 480;
        }
        else if (caerStrEquals(sourceString, "DAVIS208")) {
            sizeX = 208;
            sizeY = 192;
        }
        else {
            // Default fall-back of 640x480 (VGA).
            ofLog(OF_LOG_WARNING,
                    "Impossible to determine display sizes from Source information/string. Falling back to 640x480 (VGA).");
            sizeX = 640;
            sizeY = 480;
        }
        
    }

    
    string getUserHomeDir()
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
    
    int getdir (string dir, vector<string> &files)
    {
        DIR *dp;
        struct dirent *dirp;
        if((dp  = opendir(dir.c_str())) == NULL) {
            std::cout << "Error(" << errno << ") opening " << dir << std::endl;
            return errno;
        }
        
        while ((dirp = readdir(dp)) != NULL) {
            files.push_back(std::string(dirp->d_name));
        }
        closedir(dp);
        return 0;
    }
    
    bool forceFileMode(){
        //start file mode
        fileInput = true;
        return(true);
    }
   
    bool makeFileIndex(){
        // vector string files is in thread as well as files_id , current file
        // no need to parse header we already are at the end of it
        // no need to open file, just go trought it
        long posHeaderParsed;
        posHeaderParsed=istreamf.tellg();
        while(true){
            // make index of all timestamps
            char *buffer_header = (char*)malloc(28);
            istreamf.read(buffer_header,28);
            if( istreamf.eof() ){
                //ofLog(OF_LOG_NOTICE,"Reached the end of the file. Restarting...");
                free(buffer_header);
                break;
            }
            int eventype =  caerEventPacketHeaderGetEventType((caerEventPacketHeader)buffer_header);
            int eventsource = caerEventPacketHeaderGetEventSource((caerEventPacketHeader)buffer_header);
            int eventsize = caerEventPacketHeaderGetEventSize((caerEventPacketHeader)buffer_header);
            int eventoffset = caerEventPacketHeaderGetEventTSOffset((caerEventPacketHeader)buffer_header);
            int eventtsoverflow =  caerEventPacketHeaderGetEventTSOverflow((caerEventPacketHeader)buffer_header);
            int eventcapacity = caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader)buffer_header);
            int eventnumber = caerEventPacketHeaderGetEventNumber((caerEventPacketHeader)buffer_header);
            int eventvalid = caerEventPacketHeaderGetEventValid((caerEventPacketHeader)buffer_header);
            int next_read = eventcapacity * eventsize;
            //ofLog(OF_LOG_WARNING,"next_read %d\n", next_read);
            buffer_header = (char *)realloc(buffer_header, 28+next_read);
            istreamf.read(buffer_header+28,next_read);
            caerEventPacketContainer packetContainerT = caerEventPacketContainerAllocate(1);
            caerEventPacketContainerSetEventPacket(packetContainerT, 0, (caerEventPacketHeader)buffer_header);
            caerEventPacketContainerSetEventPacketsNumber(packetContainerT, 1);
            int32_t packetNum = caerEventPacketContainerGetEventPacketsNumber(packetContainerT);
            for (int32_t i = 0; i < packetNum; i++) {
                caerEventPacketHeader packetHeader = caerEventPacketContainerGetEventPacket(packetContainerT, i);
                if (packetHeader == NULL) {
                    //ofLog(OF_LOG_WARNING,"Packet %d is empty (not present).\n", i);
                    continue; // Skip if nothing there.
                }
                long hits = caerEventPacketContainerGetHighestEventTimestamp(packetContainerT);
                packetsHiTimestamps.push_back(hits);
                //ofLog(OF_LOG_WARNING,"Highest timestamps %lu\n", hits);
            }
            free(buffer_header);
        }
        // set the pointer back to where it was
        istreamf.clear();
        istreamf.seekg(posHeaderParsed);
        //istreamf.close();
    }

    bool tryFile(){
        // vector string files is in thread as well as files_id , current file
        
        // list all files in home
        path = getUserHomeDir();
        getdir(path,files);
        aedat_version = -1;
        
        for (unsigned int i = 0;i < files.size();i++) {
            //cout << files[i] << endl;
            // fuond aedat file
            if(files[i].substr(files[i].find_last_of(".") + 1) == "aedat") {
                string string_path = path + "/" + files[i];
                //cout << string_path << endl;
                fstream file(files[i], ios::in | ios::out | ios::binary);
                files_id = i;   // store file id
                string input_file = string_path;
                string line;
                ifstream istream;
                istream.open(input_file.c_str(),ios::binary|ios::in);
                
                // parse header
                while(getline(istream,line,'\n')){
                    if(line.empty()) continue;
                    string xx = line;
                    string header_headp = xx.erase (9,xx.length());
                    if(header_headp.compare("#!AER-DAT") == 0 ){
                        line.erase(0,9);
                        line.erase(4,line.length());
                        string version_major = line.substr(0, line.find("."));
                        size_t pos = 0;
                        string token;
                        string delimiter = ".";
                        string major_haeder;
                        string minor_header;
                        while ((pos = line.find(delimiter)) != std::string::npos) {
                            token = line.substr(0, pos);
                            //std::cout << token << std::endl;
                            minor_header = line.erase(0, pos + delimiter.length());
                        }
                        major_haeder =  token;

                        if(major_haeder.compare("3") == 0){
                            ofLog(OF_LOG_NOTICE, "Found Version 3 aedat file minor ");
                            aedat_version = 3;
                            //cout << "minor" << minor_header << endl;
                        }else{
                            ofLog(OF_LOG_ERROR, "Aedat Version not compatible found "+major_haeder+ " but we can only play version 3");
                            break;
                            
                        }

                    }
                    
                    if(aedat_version != -1){
                        fileInput = true;
                        path = string_path;

                    }
                    
                    if(line.compare("#!END-HEADER\r") == 0){
                        if(fileInput){
                            ofLog(OF_LOG_NOTICE, "File Input Enabled");
                        }
                        break;
                    }
                    
                }
                istream.close();
                
            } else {
                //cout << "Not an aedat file..." << endl;
                ;
            }
        
            
        }
    
        if(fileInput){
            return(true);
        }else{
            return(false);
        }
        
    }
    
    void threadedFunction()
    {
        
    STARTDEVICEORFILE:
        lock();
    	deviceReady = false;
        fileInput = false;
        liveInput = false;
        fileInputLocal = false;
        fileInputReady = false;
        fileIndexReady = false;
        
        // get current frame status
        apsStatus = true;
        apsStatusLocal = true;
        dvsStatus = true;
        dvsStatusLocal = true;
        imuStatus = true;
        imuStatusLocal = true;
        unlock();
    STARTFILEMODE:
        if( fileInput == false ){

            if(camera_handle == NULL){
                // start the camera
        #if DAVIS == 1
                // Open a DAVIS, give it a device ID of 1, and don't care about USB bus or SN restrictions.
                camera_handle = caerDeviceOpen(1, CAER_DEVICE_DAVIS, 0, 0, NULL);
        #endif
        #if DVS128 == 1
                // Open a DVS128, give it a device ID of 1, and don't care about USB bus or SN restrictions.
                camera_handle = caerDeviceOpen(1, CAER_DEVICE_DVS128, 0, 0, NULL);
        #endif
            }
            if (camera_handle == NULL) {
                ofLog(OF_LOG_ERROR,"error opening the device\n");
                if(tryFile()){
                    goto STARTFILEMODE;
                }
                if(forceFileMode()){
                    goto STARTFILEMODE;
                }
                //sleep(1);
                goto STARTDEVICEORFILE;
            }else{
                ofLog(OF_LOG_NOTICE,"Succesfully open a Dynamic Vision Sensor\n");
            }
            
            
            // Send the default configuration before using the device.
            // No configuration is sent automatically!
            caerDeviceSendDefaultConfig(camera_handle);
            
            // Turn on Autoexposure if device has APS
    #if DAVIS == 1
            caerDeviceConfigSet(camera_handle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_AUTOEXPOSURE, true);
    #endif
            
            // get device size
    #if DAVIS == 1
            caer_davis_info infocam = caerDavisInfoGet(camera_handle);
    #endif
    #if DVS128 == 1
            caer_dvs128_info infocam = caerDVS128InfoGet(camera_handle);
    #endif

            sizeX = infocam.dvsSizeX;
            sizeY = infocam.dvsSizeY;
            chipId = infocam.chipID;

            // Now let's get start getting some data from the device. We just loop, no notification needed.
            caerDeviceDataStart(camera_handle, NULL, NULL, NULL, NULL, NULL);

            // Let's turn on blocking data-get mode to avoid wasting resources.
            caerDeviceConfigSet(camera_handle, CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);
            
            deviceReady = true;
            
            while(isThreadRunning())
            {
                lock();
                
                packetContainerT = NULL;
                packetContainerT = caerDeviceDataGet(camera_handle);
                if (packetContainerT != NULL){
                    container.push_back(packetContainerT);
                }
                unlock();

                nanosleep((const struct timespec[]){{0, 500L}}, NULL);

                //check aps status
                if( apsStatus != apsStatusLocal){
                    apsStatusLocal = apsStatus;
                    //enable disable frames
    #if DAVIS == 1
                    caerDeviceConfigSet(camera_handle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, apsStatusLocal);
    #endif
                }
                //check dvs status
                if( dvsStatus != dvsStatusLocal){
                    dvsStatusLocal = dvsStatus;
                    //enable disable frames
    #if DAVIS == 1
                    caerDeviceConfigSet(camera_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_RUN, dvsStatusLocal);
    #elif DVS128 == 1
                    caerDeviceConfigSet(camera_handle, DVS128_CONFIG_DVS, DVS128_CONFIG_DVS_RUN, dvsStatusLocal);
    #endif
                }
                //check imu status
                if( imuStatus != imuStatusLocal){
                    imuStatusLocal = imuStatus;
                    //enable disable frames
    #if DAVIS == 1
                    caerDeviceConfigSet(camera_handle, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN, imuStatusLocal);
    #endif
                }
                
                if(fileInput){
                    goto STARTFILEMODE;
                }
                
                if(liveInput){
                    ofLog(OF_LOG_NOTICE, "trying live input \n");
                    goto STARTDEVICEORFILE;
                }

            }
        
        }else{
            
            // file input mode
            ofLog(OF_LOG_NOTICE, "Reading input file\n");
            
            doChangePath = false;
            filename_to_open = path;
            //ofLog(OF_LOG_NOTICE, "Filename "+filename_to_open);
            istreamf.open(filename_to_open.c_str(),ios::binary|ios::in);
            istreamf.seekg(0,istreamf.beg);
            if (!istreamf.is_open()){
                ofLog(OF_LOG_ERROR, "Error opening file %s", filename_to_open.c_str());
            }else{
                ofLog(OF_LOG_NOTICE, "Ok opening file %s", filename_to_open.c_str());
            }
            header_skipped = false;
            
            while(isThreadRunning())
            {
                
                lock();
                packetContainerT = NULL;
                //ofLog(OF_LOG_NOTICE, "Header skipped %d..", header_skipped);
            HEADERPARSE:
                
                if(liveInput){
                    ofLog(OF_LOG_NOTICE, "trying live input \n");
                    goto STARTDEVICEORFILE;
                }

                // parse header
                if(!header_skipped){

                    while(getline(istreamf,line,'\n')){
                        if(line.empty()) continue;
                        ofLog(OF_LOG_NOTICE, "File Header %s \n", line.c_str());
                        //set dimensions according to header file
                        char sourceString[1024 + 1];
                        if (std::sscanf(line.c_str(), "#Source %i: %1024[^\r]s\n", &chipId, sourceString) == 2) {
                            parseSourceString(sourceString);
                        }
                        if(line.compare("#!END-HEADER\r") == 0){
                            header_skipped = true;
                            ofLog(OF_LOG_NOTICE, "File Header Parsed..");
                            doChangePath = false;
                            fileInputReady = true;
                            if(fileIndexReady != true){
                                //header_skipped = false;
                                //goto STARTFILEMODE;
                                ofLog(OF_LOG_NOTICE, "Make File Index");
                                makeFileIndex(); // index timestamps
                                ofLog(OF_LOG_NOTICE, "Done Index");
                                fileIndexReady = true;
                                goto HEADERPARSE;
                            }else{
                                goto HEADERPARSE;
                            }
                        }
                    }
                    
                }
                if(header_skipped){
                    // read 28 bytes and parse file
                    //ofLog(OF_LOG_NOTICE, "Next bytes..");
                    char *buffer_header = (char*)malloc(28);
                    istreamf.read(buffer_header,28);
                    //buffer_header[28] = '0';
                    if( istreamf.eof() ){
                        ofLog(OF_LOG_NOTICE,"Reached the end of the file. Restarting...");
                        //istreamf.clear();
                        //istreamf.seekg(0, ios::beg); // from beginning
                        istreamf.close();
                        istreamf.open(filename_to_open.c_str(),ios::binary|ios::in);
                        header_skipped = false;
                        goto HEADERPARSE;
                    }
                    if (istreamf.fail( )){
                        ofLog(OF_LOG_ERROR, "Error opening aedat file..");
                        return;
                    }
                    int eventype =  caerEventPacketHeaderGetEventType((caerEventPacketHeader)buffer_header);
                    int eventsource = caerEventPacketHeaderGetEventSource((caerEventPacketHeader)buffer_header);
                    int eventsize = caerEventPacketHeaderGetEventSize((caerEventPacketHeader)buffer_header);
                    int eventoffset = caerEventPacketHeaderGetEventTSOffset((caerEventPacketHeader)buffer_header);
                    int eventtsoverflow =  caerEventPacketHeaderGetEventTSOverflow((caerEventPacketHeader)buffer_header);
                    int eventcapacity = caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader)buffer_header);
                    int eventnumber = caerEventPacketHeaderGetEventNumber((caerEventPacketHeader)buffer_header);
                    int eventvalid = caerEventPacketHeaderGetEventValid((caerEventPacketHeader)buffer_header);
                    int next_read = eventcapacity * eventsize;
                    buffer_header = (char *)realloc(buffer_header, 28+next_read);
                    istreamf.read(buffer_header+28,next_read);
                    packetContainerT = caerEventPacketContainerAllocate(1);
                    caerEventPacketContainerSetEventPacket(packetContainerT, 0, (caerEventPacketHeader)buffer_header);
                    caerEventPacketContainerSetEventPacketsNumber(packetContainerT, 1);
                }
                if (packetContainerT != NULL){
                    container.push_back(packetContainerT);
                }
                if(doChangePath){
                    filename_to_open = path;
                    ofLog(OF_LOG_NOTICE, "Filename "+filename_to_open);
                    istreamf.open(filename_to_open.c_str(),ios::binary|ios::in);
                    header_skipped = false;
                    doChangePath = false;
                    fileIndexReady = false;
                    packetsHiTimestamps.clear();
                    packetsHiTimestamps.shrink_to_fit();
                    goto HEADERPARSE;
                }
                unlock();
                nanosleep((const struct timespec[]){{0, 5000L}}, NULL);
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
    bool fileInputLocal;

    //size device
    int sizeX;
    int sizeY;
    bool deviceReady;
    int chipId;
    bool liveInput;
    
    //file input
    vector<string> files = vector<string>();
    int files_id;
    bool fileInput;
    int aedat_version;
    bool fileInputReady;
    string path;
    bool doChangePath;
    bool header_skipped;
    ifstream istreamf;
    vector<long> packetsHiTimestamps;
    bool fileIndexReady;
    
    //tmp
    string line;
    string filename_to_open;
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
    void changeStats();
    void drawImageGenerator();
    const char * chipIDToName(int16_t chipID, bool withEndSlash);
    void writeHeaderFile();
    static int packetsFirstTimestampThenTypeCmp(const void *a, const void *b);
    void changeRecordingStatus();
    void openRecordingFile();
    string getUserHomeDir();
    void loadFile();
    void initThreadVariables();
    void tryLive();

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
    vector<ofImage> packetsImageGenerator;
    caerEventPacketContainer packetContainer;
    
    // Data functions
    vector<polarity> getPolarity();
    vector<frame> getFrames();
    ofImage getImageGenerator();
    void initImageGenerator();
    void updateImageGenerator();
    void initBAfilter();
    void updateBAFilter();
    long **baFilterMap;
    void changePath();
    void changeTargetSpeed(long val);
    long getTargetSpeed();
    void changePause();
    
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
    bool statsStatus; // enable/disable stats

    //size
    int sizeX;
    int sizeY;
    int chipId;
    
    //Image Generator
    ofImage imageGenerator;
    float** spikeFeatures;
    bool rectifyPolarities;
    int numSpikes;
    int counterSpikes;
    //File system
    int isRecording;
    string path;
    bool doChangePath;
    bool header_skipped;
    bool paused;
    
    //file output aedat 3.0
    ofstream myFile;
    vector<long> packetsHiTimestamps;
    vector<long> ofxTime;
    long ofxLastTs;
    long targetSpeed;
};


#endif /* ofxDVS_hpp */
