//
//  ofxDVS.hpp
//  ofxDVS
//
//  Created by Federico Corradi on 19.05.17.
// // udpated 2024
//

#ifndef ofxDVS_hpp
#define ofxDVS_hpp


#include "libcaer.h"
#include "devices/davis.h"
#include "devices/dvs128.h"
#include "devices/dvxplorer.h"
#include "libcaer/devices/device_discover.h"
#include <dv-processing/io/camera/discovery.hpp>

#include <atomic>


#include "ofxDvsPolarity.hpp"
#include "RectangularClusterTracker.hpp"

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

#include <vector>
#include <fstream>
#include <cerrno>
#include <ctime>   // nanosleep() on POSIX

#include <limits>
#include <iomanip>

/// PLEASE SELECT SENSOR DAVIS or DVS128
#define DAVIS  0
#define DVS128 0
#define DVXPLORER 1

// and decide on parameters
#define DEBUG 0

#include "ofMain.h"
#include "ofxDatGui.h"

// in ofxDVS.hpp
#include "onnx_run.hpp"

struct polarity {
    int info;
    ofPoint pos;
    int timestamp;
    bool pol;
    bool valid;
};

// --- new helpers at the top of ofxDVS.hpp or in a small .cpp ---

// Convert dv-processing events into your 'polarity' struct
static inline void appendPolarityBatch(
    const dv::EventStore &events,
    std::vector<polarity> &out)
{
    out.reserve(out.size() + events.size());
    for (const auto &e : events) {
        polarity p{};
        p.info      = 0;
        p.pos.x     = static_cast<float>(e.x());
        p.pos.y     = static_cast<float>(e.y());
        p.timestamp = static_cast<int>(e.timestamp()); // you can keep it as 32-bit for now
        p.pol       = e.polarity();
        p.valid     = true;
        out.push_back(p);
    }
}


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
    float temperature;
};




/*class alphaThread: public ofThread
{
public:
    
    //--------------------------
    void initVisualizerMapTh(){
        visualizerMap=new float*[sizeX];
        for( int i=0; i<sizeX; ++i ) {
            visualizerMap[i] = new float[sizeY];
            for( int j=0; j<sizeY; ++j ) {
                visualizerMap[i][j] -= 0.02;
                if(visualizerMap[i][j] < 0){
                    visualizerMap[i][j] = 0;
                }
            }
        }
    }
    
    //--------------------------
    void threadedFunction()
    {
        // wait for size
        LOCK_CHECK_ALPHA:
        lock();
        if(init != true){
            unlock();
            goto LOCK_CHECK_ALPHA;
        }
        initVisualizerMapTh();
        unlock();

        while(isThreadRunning())
        {
            lock();
            // decay map
            for (int x = 0; x < sizeX; x++) {
                for (int y = 0; y < sizeY; y++) {
                  //visualizerMap[x][y] = 0;
                    ;
                }
            }
            unlock();
            // not busy loop
            nanosleep((const struct timespec[]){{0, 5000000L}}, NULL);
        }
    }
    
    float **visualizerMap;
    int sizeX;
    int sizeY;
    int fsint;
    bool init = false;
};*/


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
        /*else if (caerStrEquals(sourceString, "DVXPLORER")) {
            sizeX = 640;
            sizeY = 480;
        }*/
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
                //packetsHiTimestamps.push_back(hits);
                //ofLog(OF_LOG_WARNING,"Highest timestamps %lu\n", hits);
            }
            free(buffer_header);
        }
        // set the pointer back to where it was
        istreamf.clear();
        istreamf.seekg(posHeaderParsed);
	return true;
    }

    bool tryFile(){
        // vector string files is in thread as well as files_id , current file
        
        // list all files in home
        path = getUserHomeDir();
        getdir(path,files);
        aedat_version = -1;
        
        for (unsigned int i = 0;i < files.size();i++) {
            //cout << files[i] << endl;
            // found aedat file
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
        extInputStatus = false;
        extInputStatusLocal = false;
        resetTsStatus = true;

        unlock();
    STARTFILEMODE:
        // ---------- Prefer LIVE CAMERA ----------
        if (fileInput == false) {
            try {
                // EITHER open the first available DV camera:
                auto cam = dv::io::camera::open();

                std::cout << "Opened: " << cam->getCameraName() << "\n";
                if (cam->isEventStreamAvailable()) {
                    auto res = cam->getEventResolution().value();
                    std::cout << "Event resolution: " << res << "\n";
                }

                // OR, since you have a DVXplorer Mini, open it explicitly:
                //auto cam = std::make_unique<dv::io::camera::DVXplorerM>();

                if (!cam || !cam->isRunning()) {
                    ofLogError() << "No DV camera found or failed to start.";
                    // fall back to file mode if you want
                    if (tryFile()) goto STARTDEVICEORFILE;
                    goto STARTDEVICEORFILE;
                }

                // Set device size for your drawing code
                if (cam->isEventStreamAvailable()) {
                    auto res = cam->getEventResolution().value();
                    sizeX = res.width;
                    sizeY = res.height;
                } else {
                    // sane defaults
                    sizeX = 640; sizeY = 480;
                }

                deviceReady = true;

                // Main loop: pull event batches and push to your containers
                while (isThreadRunning() && cam->isRunning()) {
                    // Toggle requests you had (local flags) can be handled here:
                    // (dv-processing doesn’t have “run DVS on/off” for DVXplorer,
                    //  but you can just ignore incoming events if dvsStatusLocal == false)

                    if (auto events = cam->getNextEventBatch(); events.has_value()) {

                        // You used to push 'packetContainerT' – now push directly what you want to draw
                        // For minimal changes, fill a temporary vector of polarity and store it where your UI expects it.
                        // Example: push into a per-iteration buffer 'packetsPolarity' (make sure it's cleared somewhere).
                        //if (auto events = cam->getNextEventBatch(); events.has_value()) {

                        // 1) Allocate a libcaer polarity packet with capacity = number of events
                        const int32_t capacity = static_cast<int32_t>(events->size());
                        // tsOverflow=0 (fine for short runs); source=1 (pick what you prefer)
                        caerPolarityEventPacket polPkt = caerPolarityEventPacketAllocate(capacity, /*tsOverflow*/0, /*source*/1);
                        if (polPkt != nullptr) {
                            // 2) Fill the packet
                            int32_t idx = 0;
                            for (const auto &ev : *events) {
                                caerPolarityEvent e = caerPolarityEventPacketGetEvent(polPkt, idx);

                                // NOTE: libcaer timestamps are 32-bit; dv-processing uses 64-bit.
                                // For short sessions we can safely truncate. If you want robust handling,
                                // compute and set overflow in the header (see note below).
                                const int32_t ts32 = static_cast<int32_t>(ev.timestamp() & 0x7fffffff);

                                caerPolarityEventSetTimestamp(e, ts32);
                                caerPolarityEventSetX(e, static_cast<uint16_t>(ev.x()));
                                caerPolarityEventSetY(e, static_cast<uint16_t>(ev.y()));
                                caerPolarityEventSetPolarity(e, ev.polarity());
                                caerPolarityEventValidate(e, polPkt);
                                ++idx;
                                //ofLogError() << "some packets.";
                            }

                            // 3) Finalize header: number of events = valid = idx
                            caerEventPacketHeaderSetEventNumber(&polPkt->packetHeader, idx);
                            caerEventPacketHeaderSetEventValid(&polPkt->packetHeader, idx);

                            // 4) Wrap into a libcaer container (exactly as your code expects)
                            caerEventPacketContainer cont = caerEventPacketContainerAllocate(1);
                            caerEventPacketContainerSetEventPacket(cont, 0, (caerEventPacketHeader)polPkt);

                            // 5) Push to your existing queue (you already free it later in your pipeline)
                            // Now lock briefly just to push.
                            lock();
                            // Optional: cap the queue to avoid UI backlog.
                            constexpr size_t MAX_QUEUE = 15;
                            if (container.size() >= MAX_QUEUE) {
                                // drop oldest
                                caerEventPacketContainerFree(container.front());
                                container.erase(container.begin());
                            }
                            container.push_back(cont);
                            unlock();
                        }       
                        nanosleep((const struct timespec[]){{0, 500L}}, NULL);
                            
                    } else {
                        // nothing arrived; small sleep avoids busy spin
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        //std::cout << "Nanosleep: "  << "\n";
                    }

                    // Handle your own mode switches
                    if (fileInput) {         // you flipped to file mode elsewhere
                        cam.reset();
                        goto STARTDEVICEORFILE;
                    }
                    if (liveInput) {         // request to re-open live
                        cam.reset();
                        goto STARTDEVICEORFILE;
                    }
                }

                cam.reset();
            }
            catch (const std::exception &e) {
                ofLogError() << "Camera exception: " << e.what();
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                goto STARTDEVICEORFILE;
            }
        }else{
            
            // file input mode
            ofLog(OF_LOG_NOTICE, "Reading input file\n");
            doLoad = true;
            
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

              PAUSED:
				if(paused){
					nanosleep((const struct timespec[]){{0, 50000L}}, NULL);
					goto PAUSED;
				}

                if(liveInput){
                    ofLog(OF_LOG_NOTICE, "trying live input \n");
                    goto STARTDEVICEORFILE;
                }

                lock(); // thread lock
              HEADERPARSE:
                packetContainerT = NULL;

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
                if(header_skipped && doLoad){
                    // read 28 bytes and parse file
                    //ofLog(OF_LOG_NOTICE, "Next bytes..");
                    char *buffer_header = (char*)malloc(28);
                    istreamf.read(buffer_header,28);
                    if( istreamf.eof() ){
                        ofLog(OF_LOG_NOTICE,"Reached the end of the file. Restarting...");
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
                    //packetsHiTimestamps.clear();
                    //packetsHiTimestamps.shrink_to_fit();
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
    bool extInputStatus;
    bool extInputStatusLocal;
    bool resetTsStatus;

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
    bool fileIndexReady;
    bool paused;
    bool doLoad;
    
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
    void drawFixed();
    void drawSpikes();
    void updateMeshSpikes();
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
    void changeRecordingStatusDb(); // for radar
    void openRecordingFile();
    void openRecordingFileDb(); // for radar
    void changeLoadFile();
    bool orginal; // dataset or selected
    string getUserHomeDir();
    void loadFile();
    void initThreadVariables();
    void tryLive();
    void changeColor(int i);
    void setIMU(bool value);
    void setAPS(bool value);
	void setDVS(bool value);
	void setPause(bool value);
	void setExtInput(bool value);
	void resetTs();


    // Camera
    std::atomic_bool globalShutdown = ATOMIC_VAR_INIT(false);
    void globalShutdownSignalHandler(int signal);
    
    // Textures and framebuffer
    ofFbo fbo;
    ofTexture* tex;
    ofTexture* getTextureRef();
    // easycam
    ofEasyCam myCam;
    ofMesh mesh;
    ofImage imagePolarity;
    bool newImagePol;
    
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
    void setDrawImageGen(bool doDraw);
    bool getDrawImageGen();
    void initBAfilter();
    void updateBAFilter();
    void initVisualizerMap();
    long **baFilterMap;
    void changePath();
    void changeTargetSpeed(float val);
    float getTargetSpeed();
    void setTargetSpeed(float value);
    void changePause();
    void clearDraw();
    float **visualizerMap;
    void changeFSInt(float i);
    void changeBAdeltat(float i);
    void setImageAccumulatorSpikes(float i);
    void setImageAccumulatorDecay(float i);
    void setDrawSpikes(bool doDraw);
    bool getDrawSpikes();
    void set3DTime(int i);
    void drawMouseDistanceToSpikes();
    void setPointer(bool i);
    void setDrawImu(bool i);
    void initMeanRate();
    float getImuTemp();
    
    // color palette for spikes
    int spkOnR[8];
    int spkOnG[8];
    int spkOnB[8];
    int spkOnA;
    int spkOffR[8];
    int spkOffG[8];
    int spkOffB[8];
    int spkOffA;
    int paletteSpike;
    int maxContainerQueued;
    float fsint;
    bool liveInput;
    bool doDrawSpikes;
    
    // thread usb
    usbThread thread;
    bool apsStatus; // enable/disable aps
    bool dvsStatus; // enable/disable dvs
    bool imuStatus; // enable/disable imu
    bool statsStatus; // enable/disable stats
    
    // alpha usb
    //alphaThread thread_alpha;
    
    //size
    int sizeX;
    int sizeY;
    int chipId;
    
    //Image Generator
    ofImage imageGenerator;
    float** spikeFeatures;
    float** surfaceMapLastTs;
    bool rectifyPolarities;
    float numSpikes;
    int counterSpikes;
    bool drawImageGen;
    float decaySpikeFeatures;
    bool newImageGen;
    
    //Mean Rate
    ofImage meanRateImage;
    float** frequencyMap;
    float** spikeCountMap;
    bool startedMeas;
    float measureMinTime;
    float measureStartedAt;
    
    // Ba filter
    float BAdeltaT;
    
    //File system
    int isRecording;
    string path;
    bool doChangePath;
    bool header_skipped;
    bool paused;
    
    //file output aedat 3.0
    ofstream myFile;
    //vector<long> packetsHiTimestamps;
    vector<long> ofxTime;
    long ofxLastTs;
    float targetSpeed;   // real time speed
    
    // timing information
    long current;
    long started;
    bool isStarted;
    long microseconds;
    long seconds;
    long minutes;
    long hours;
    char timeString[256];
    // 
 /*   long timestampcurrentelapsedlive;
    long timestampcurrentelapsedlive_buf;
    long timestampcurrentelapsedfile;
    long timestampcurrentelapsedfile_0;
    long timestampcurrentelapsedfile_1;
    long timestampcurrentelapsedfile_buf;
    bool tip;*/
    
    // imu temperature
    float imuTemp;
    
    // mesh
    long tmp;
    long m;
    long nus;
    
    bool drawDistanceMesh;
    string chipName;
    bool doDrawImu6;
    
    // cam rotation - translation
    ofQuaternion rotationCam;
    ofVec3f translationCam;
    ofVec3f cameraPos;
    
    //Gui
    string getHex(int hex);
    void changeDrawGui();
    ofxDatGuiFolder* f1;
    void onButtonEvent(ofxDatGuiButtonEvent e);
    void onToggleEvent(ofxDatGuiToggleEvent e);
    void onSliderEvent(ofxDatGuiSliderEvent e);
    void onMatrixEvent(ofxDatGuiMatrixEvent e);
    void on2dPadEvent(ofxDatGui2dPadEvent e);
    void onTextInputEvent(ofxDatGuiTextInputEvent e);
    void onColorPickerEvent(ofxDatGuiColorPickerEvent e);
    ofxDatGuiTextInput * myTextTimer;
    ofxDatGuiTextInput * myTempReader;
    ofxDatGuiValuePlotter * myIMU;
    bool drawGui;
    void keyPressed(int key);
    
    //counters
    int numPaused;
    int numPausedRec;
        

    //tracker DVS
    void    createRectangularClusterTracker();
    void enableTracker(bool enabled);
    std::unique_ptr<RectangularClusterTracker> rectangularClusterTracker;
    bool rectangularClusterTrackerEnabled = false;

    RectangularClusterTracker::Config rectangularClusterTrackerConfig;

    void onTrackerToggleEvent(ofxDatGuiToggleEvent e);
    void onTrackerSliderEvent(ofxDatGuiSliderEvent e);    
    void setEnabledDvsSensorGuiControls(bool new_state);
    void setEnabledDvsSensorConfigGuiControls(bool new_state);

    std::unique_ptr<ofxDatGui>  tracker_panel;

    void mousePressed(int x, int y, int button);
    

    /* visualisation primitives we use
     */

    ofTexture       next_frame;
    ofTexture       next_polarities;
    ofPixels        next_polarities_pixbuf;
    ofMesh          next_polarities_3d;
    ofEasyCam       camera;
    ofRectangle     frame_viewport;
    ofRectangle     polarities_viewport;
    ofRectangle     cam_viewport;
    void updateViewports();

    //
    std::unique_ptr<OnnxRunner> nn;
    bool nnEnabled = false;
    int  nnLastClass = -1;


    //yolo
    // ---- YOLO overlay ----
    struct YoloDet { ofRectangle box; float score; int cls; };
    std::vector<YoloDet> yolo_dets;
    float yolo_conf_thresh = 0.8f;
    float yolo_iou_thresh  = 0.45f;
    int   yolo_input_w = 640, yolo_input_h = 640; // filled from model on first run
    bool  yolo_draw = true;                       // draw overlay when true
    // optional extra knobs (if not already defined)
    bool  yolo_show_labels = true;
    int   yolo_smooth_frames = 2;   // history length for temporal smoothing (1..5)   

    // spikevision
    std::unique_ptr<OnnxRunner> tsdt;   // a second runner, independent of YOLO
    bool tsdtEnabled = false;           // panel toggle
    bool tsdt_show_label = true;        // draw overlay text
    // model I/O
    int  tsdt_T = 8;                     // timesteps (your export used 8)
    int  tsdt_inH = 128, tsdt_inW = 128; // DVS128
    int  tsdt_bin_ms = 10;               // per-timestep bin width (8 * 6ms ~= 48ms window)
    long tsdt_bin_us = (long)tsdt_bin_ms * 1000L;
    int tsdt_ev_per_bin = 10000;
    // event history (last T bins)
    struct TsEvent { int x, y; bool p; long ts; };
    std::deque<TsEvent> tsdt_hist;      // pruned each frame to last T*bin_us window
    // last prediction
    std::vector<std::string> tsdt_labels = {
        "hand_clapping","right_hand_wave","left_hand_wave","right_hand_clockwise","right_hand_counter_clockwise",
        "left_hand_clockwise","left_hand_counter_clockwise","forearm_roll_forward/backward","guitar","random_other_gestures"
    };
    int   tsdt_last_idx  = -1;
    float tsdt_last_conf = 0.f;
    // small EMA smoothing so label doesn’t flicker
    float tsdt_ema_alpha = 1.0f;
    std::vector<float> tsdt_ema_logits; // same length as labels
    void drawTsdtLabelBottomCenter();
    void tsdtSelfTest();
    void runTsdtDebugFromFile(OnnxRunner* tsdt);

    // --- NN / YOLO panel ---
    std::unique_ptr<ofxDatGui> nn_panel;

    void onNNToggleEvent(ofxDatGuiToggleEvent e);
    void onNNSliderEvent(ofxDatGuiSliderEvent e);
    void onNNButtonEvent(ofxDatGuiButtonEvent e);

    // VTEI window (GUI-controlled)
    float vtei_win_ms = 50.0f;   // default 50 ms
    long  vtei_win_us = 50000;   // = vtei_win_ms * 1000

    void setVteiWindowMs(float ms) {
        vtei_win_ms = std::max(1.0f, ms);                    // clamp to >=1 ms
        vtei_win_us = static_cast<long>(vtei_win_ms * 1000); // keep µs in sync
    }

    // --- overlays ---
    void drawRectangularClusterTracker();
    void drawYoloDetections();


private:
    // Build VTEI (5 channels) in CHW order at (W,H)
    std::vector<float> buildVTEI_(int W, int H);
    std::deque<std::vector<YoloDet>> yolo_hist_;
    int yolo_hist_len_ = 2;

    // --- Hot pixel suppression ---
    int hot_refrac_us = 200;                  // ignore events from same pixel if dt < this
    std::vector<int64_t> last_ts_map_;        // sizeX*sizeY
    void applyRefractory_();

    //point shader
    ofShader pointShader;
    float pointSizePx = 8.0f;   // tweak at runtime if you like

    // packet queue managment
    std::deque<caerEventPacketContainer> backlog_;
    size_t backlog_max_ = 15; // small, to bound latency

};


#endif /* ofxDVS_hpp */
