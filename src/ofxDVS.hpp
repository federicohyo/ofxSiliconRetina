//
//  ofxDVS.hpp
//  ofxDVS
//
//  Created by Federico Corradi on 19.05.17.
//  Updated 2024-2026
//

#ifndef ofxDVS_hpp
#define ofxDVS_hpp


#include "libcaer.h"
#include "devices/davis.h"
#include "devices/dvs128.h"
#include "devices/dvxplorer.h"
#include "libcaer/devices/device_discover.h"
#include <dv-processing/io/camera/discovery.hpp>
#include <dv-processing/io/mono_camera_writer.hpp>
#include <dv-processing/io/mono_camera_recording.hpp>

#include <atomic>

enum class AedatFormat { UNKNOWN, AEDAT31, AEDAT4 };


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
#include <ctime>

#include <limits>
#include <iomanip>

/// PLEASE SELECT SENSOR DAVIS or DVS128
#define DAVIS  0
#define DVS128 0
#define DVXPLORER 1

#define DEBUG 0

#include "ofMain.h"
#include "ofxDatGui.h"
#include "ofxFFmpegRecorder.h"

// Pipeline headers
#include "onnx_run.hpp"
#include "dvs_nn_utils.hpp"
#include "dvs_yolo_pipeline.hpp"
#include "dvs_tsdt_pipeline.hpp"
#include "dvs_inference_worker.hpp"

struct polarity {
    int info;
    ofPoint pos;
    int64_t timestamp;
    bool pol;
    bool valid;
};

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
        p.timestamp = static_cast<int>(e.timestamp());
        p.pol       = e.polarity();
        p.valid     = true;
        out.push_back(p);
    }
}


struct frame {
    int info;
    int frameStart;
    int frameEnd;
    int exposureStart;
    int exposureEnd;
    int lenghtX;
    int lenghtY;
    int positionX;
    int positionY;
    enum caer_frame_event_color_channels frameChannels;
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


class usbThread: public ofThread
{
public:

    void parseSourceString(char *sourceString) {
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
        fileInput = true;
        return(true);
    }

    bool makeFileIndex(){
        long posHeaderParsed;
        posHeaderParsed=istreamf.tellg();
        while(true){
            char *buffer_header = (char*)malloc(28);
            istreamf.read(buffer_header,28);
            if( istreamf.eof() ){
                free(buffer_header);
                break;
            }
            int eventsize = caerEventPacketHeaderGetEventSize((caerEventPacketHeader)buffer_header);
            int eventcapacity = caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader)buffer_header);
            int next_read = eventcapacity * eventsize;
            buffer_header = (char *)realloc(buffer_header, 28+next_read);
            istreamf.read(buffer_header+28,next_read);
            caerEventPacketContainer packetContainerT = caerEventPacketContainerAllocate(1);
            caerEventPacketContainerSetEventPacket(packetContainerT, 0, (caerEventPacketHeader)buffer_header);
            caerEventPacketContainerSetEventPacketsNumber(packetContainerT, 1);
            int32_t packetNum = caerEventPacketContainerGetEventPacketsNumber(packetContainerT);
            for (int32_t i = 0; i < packetNum; i++) {
                caerEventPacketHeader packetHeader = caerEventPacketContainerGetEventPacket(packetContainerT, i);
                if (packetHeader == NULL) continue;
            }
            free(buffer_header);
        }
        istreamf.clear();
        istreamf.seekg(posHeaderParsed);
	return true;
    }

    bool tryFile(){
        path = getUserHomeDir();
        getdir(path,files);
        aedat_version = -1;

        for (unsigned int i = 0;i < files.size();i++) {
            std::string ext = files[i].substr(files[i].find_last_of(".") + 1);

            // AEDAT4 files — no header parsing needed
            if (ext == "aedat4") {
                string string_path = path + "/" + files[i];
                files_id = i;
                fileInput = true;
                fileFormat = AedatFormat::AEDAT4;
                path = string_path;
                ofLog(OF_LOG_NOTICE, "Found AEDAT4 file: %s", string_path.c_str());
                return fileInput;
            }

            // AEDAT 3.1 files — parse header for version check
            if(ext == "aedat") {
                string string_path = path + "/" + files[i];
                fstream file(files[i], ios::in | ios::out | ios::binary);
                files_id = i;
                string input_file = string_path;
                string line;
                ifstream istream;
                istream.open(input_file.c_str(),ios::binary|ios::in);

                while(getline(istream,line,'\n')){
                    if(line.empty()) continue;
                    string xx = line;
                    string header_headp = xx.erase (9,xx.length());
                    if(header_headp.compare("#!AER-DAT") == 0 ){
                        line.erase(0,9);
                        line.erase(4,line.length());
                        size_t pos = 0;
                        string token;
                        string delimiter = ".";
                        string major_haeder;
                        while ((pos = line.find(delimiter)) != std::string::npos) {
                            token = line.substr(0, pos);
                            line.erase(0, pos + delimiter.length());
                        }
                        major_haeder = token;

                        if(major_haeder.compare("3") == 0){
                            ofLog(OF_LOG_NOTICE, "Found Version 3 aedat file");
                            aedat_version = 3;
                        }else{
                            ofLog(OF_LOG_ERROR, "Aedat Version not compatible found "+major_haeder);
                            break;
                        }
                    }

                    if(aedat_version != -1){
                        fileInput = true;
                        fileFormat = AedatFormat::AEDAT31;
                        path = string_path;
                    }

                    if(line.compare("#!END-HEADER\r") == 0){
                        if(fileInput) ofLog(OF_LOG_NOTICE, "File Input Enabled");
                        break;
                    }
                }
                istream.close();
            }
        }

        return fileInput;
    }

    void threadedFunction()
    {
    STARTDEVICEORFILE:
        lock();
    	deviceReady = false;
        // NOTE: fileInput is NOT reset here — it is a mode flag set
        // externally by changePath()/tryFile()/tryLive().
        liveInput = false;
        fileInputLocal = false;
        fileInputReady = false;
        fileIndexReady = false;

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
        if (fileInput == false) {
            try {
                auto cam = dv::io::camera::open();
                std::cout << "Opened: " << cam->getCameraName() << "\n";
                if (cam->isEventStreamAvailable()) {
                    auto res = cam->getEventResolution().value();
                    std::cout << "Event resolution: " << res << "\n";
                }

                if (!cam || !cam->isRunning()) {
                    ofLogError() << "No DV camera found or failed to start.";
                    if (tryFile()) goto STARTDEVICEORFILE;
                    goto STARTDEVICEORFILE;
                }

                if (cam->isEventStreamAvailable()) {
                    auto res = cam->getEventResolution().value();
                    sizeX = res.width;
                    sizeY = res.height;
                } else {
                    sizeX = 640; sizeY = 480;
                }

                deviceReady = true;

                while (isThreadRunning() && cam->isRunning()) {
                    if (auto events = cam->getNextEventBatch(); events.has_value()) {
                        const int32_t capacity = static_cast<int32_t>(events->size());
                        caerPolarityEventPacket polPkt = caerPolarityEventPacketAllocate(capacity, 0, 1);
                        if (polPkt != nullptr) {
                            int32_t idx = 0;
                            for (const auto &ev : *events) {
                                caerPolarityEvent e = caerPolarityEventPacketGetEvent(polPkt, idx);
                                const int32_t ts32 = static_cast<int32_t>(ev.timestamp() & 0x7fffffff);
                                caerPolarityEventSetTimestamp(e, ts32);
                                caerPolarityEventSetX(e, static_cast<uint16_t>(ev.x()));
                                caerPolarityEventSetY(e, static_cast<uint16_t>(ev.y()));
                                caerPolarityEventSetPolarity(e, ev.polarity());
                                caerPolarityEventValidate(e, polPkt);
                                ++idx;
                            }
                            caerEventPacketHeaderSetEventNumber(&polPkt->packetHeader, idx);
                            caerEventPacketHeaderSetEventValid(&polPkt->packetHeader, idx);

                            caerEventPacketContainer cont = caerEventPacketContainerAllocate(1);
                            caerEventPacketContainerSetEventPacket(cont, 0, (caerEventPacketHeader)polPkt);

                            lock();
                            constexpr size_t MAX_QUEUE = 15;
                            if (container.size() >= MAX_QUEUE) {
                                caerEventPacketContainerFree(container.front());
                                container.erase(container.begin());
                            }
                            container.push_back(cont);
                            unlock();
                        }
                        nanosleep((const struct timespec[]){{0, 500L}}, NULL);
                    } else {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    }

                    if (fileInput) { cam.reset(); goto STARTDEVICEORFILE; }
                    if (liveInput) { cam.reset(); goto STARTDEVICEORFILE; }
                }
                cam.reset();
            }
            catch (const std::exception &e) {
                ofLogError() << "Camera exception: " << e.what();
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                goto STARTDEVICEORFILE;
            }
        }else{
            ofLog(OF_LOG_NOTICE, "Reading input file\n");
            doLoad = true;
            doChangePath = false;
            filename_to_open = path;

            // Detect format by extension
            {
                std::string ext;
                auto dotPos = filename_to_open.rfind('.');
                if (dotPos != std::string::npos)
                    ext = filename_to_open.substr(dotPos);
                if (ext == ".aedat4")
                    fileFormat = AedatFormat::AEDAT4;
                else
                    fileFormat = AedatFormat::AEDAT31;
            }

        SELECTFORMAT:

            // =============== AEDAT4 reading path ===============
            if (fileFormat == AedatFormat::AEDAT4) {
                try {
                    aedat4Reader = std::make_unique<dv::io::MonoCameraRecording>(filename_to_open);
                    if (aedat4Reader->isEventStreamAvailable()) {
                        auto res = aedat4Reader->getEventResolution().value();
                        sizeX = res.width;
                        sizeY = res.height;
                    }
                    fileInputReady = true;
                    ofLog(OF_LOG_NOTICE, "Opened AEDAT4 file: %s (%dx%d)",
                          filename_to_open.c_str(), sizeX, sizeY);
                } catch (const std::exception &ex) {
                    ofLog(OF_LOG_ERROR, "Failed to open AEDAT4: %s", ex.what());
                    goto STARTDEVICEORFILE;
                }

                while (isThreadRunning()) {
                    if (paused) {
                        nanosleep((const struct timespec[]){{0, 50000L}}, NULL);
                        continue;
                    }
                    if (liveInput) {
                        aedat4Reader.reset();
                        goto STARTDEVICEORFILE;
                    }
                    if (doChangePath) {
                        aedat4Reader.reset();
                        filename_to_open = path;
                        // Detect new format
                        std::string ext2;
                        auto dp2 = filename_to_open.rfind('.');
                        if (dp2 != std::string::npos) ext2 = filename_to_open.substr(dp2);
                        if (ext2 != ".aedat4") {
                            fileFormat = AedatFormat::AEDAT31;
                            doChangePath = false;
                            goto SELECTFORMAT;
                        }
                        try {
                            aedat4Reader = std::make_unique<dv::io::MonoCameraRecording>(filename_to_open);
                            if (aedat4Reader->isEventStreamAvailable()) {
                                auto res = aedat4Reader->getEventResolution().value();
                                sizeX = res.width;
                                sizeY = res.height;
                            }
                            fileInputReady = true;
                            resetTimingFlag.store(true);
                            doChangePath = false;
                        } catch (const std::exception &ex) {
                            ofLog(OF_LOG_ERROR, "Failed to reopen AEDAT4: %s", ex.what());
                            doChangePath = false;
                        }
                        continue;
                    }

                    auto events = aedat4Reader->getNextEventBatch();
                    if (!events.has_value()) {
                        // EOF — loop the file
                        ofLog(OF_LOG_NOTICE, "AEDAT4: Reached end of file. Restarting...");
                        aedat4Reader.reset();
                        aedat4Reader = std::make_unique<dv::io::MonoCameraRecording>(filename_to_open);
                        resetTimingFlag.store(true);
                        continue;
                    }

                    // Convert dv::EventStore to caerPolarityEventPacket
                    const int32_t capacity = static_cast<int32_t>(events->size());
                    if (capacity == 0) continue;
                    caerPolarityEventPacket polPkt = caerPolarityEventPacketAllocate(capacity, 0, 1);
                    if (polPkt == nullptr) continue;

                    int32_t idx = 0;
                    for (const auto &ev : *events) {
                        caerPolarityEvent e = caerPolarityEventPacketGetEvent(polPkt, idx);
                        const int32_t ts32 = static_cast<int32_t>(ev.timestamp() & 0x7fffffff);
                        caerPolarityEventSetTimestamp(e, ts32);
                        caerPolarityEventSetX(e, static_cast<uint16_t>(ev.x()));
                        caerPolarityEventSetY(e, static_cast<uint16_t>(ev.y()));
                        caerPolarityEventSetPolarity(e, ev.polarity());
                        caerPolarityEventValidate(e, polPkt);
                        ++idx;
                    }
                    caerEventPacketHeaderSetEventNumber(&polPkt->packetHeader, idx);
                    caerEventPacketHeaderSetEventValid(&polPkt->packetHeader, idx);

                    caerEventPacketContainer cont = caerEventPacketContainerAllocate(1);
                    caerEventPacketContainerSetEventPacket(cont, 0, (caerEventPacketHeader)polPkt);

                    // Backpressure: wait when queue is full (don't drop packets)
                    lock();
                    constexpr size_t MAX_QUEUE = 64;
                    while (container.size() >= MAX_QUEUE &&
                           isThreadRunning() && !liveInput && !doChangePath && !paused) {
                        unlock();
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        lock();
                    }
                    container.push_back(cont);
                    unlock();

                    nanosleep((const struct timespec[]){{0, 500L}}, NULL);
                }
            }
            // =============== AEDAT 3.1 reading path (backward compatible) ===============
            else {
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

                lock();
              HEADERPARSE:
                packetContainerT = NULL;

                if(!header_skipped){
                    while(getline(istreamf,line,'\n')){
                        if(line.empty()) continue;
                        ofLog(OF_LOG_NOTICE, "File Header %s \n", line.c_str());
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
                                ofLog(OF_LOG_NOTICE, "Make File Index");
                                makeFileIndex();
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
                    char *buffer_header = (char*)malloc(28);
                    istreamf.read(buffer_header,28);
                    if( istreamf.eof() ){
                        ofLog(OF_LOG_NOTICE,"Reached the end of the file. Restarting...");
                        istreamf.close();
                        istreamf.open(filename_to_open.c_str(),ios::binary|ios::in);
                        header_skipped = false;
                        resetTimingFlag.store(true);
                        goto HEADERPARSE;
                    }
                    if (istreamf.fail()){
                        ofLog(OF_LOG_ERROR, "Error opening aedat file..");
                        return;
                    }
                    int eventsize = caerEventPacketHeaderGetEventSize((caerEventPacketHeader)buffer_header);
                    int eventcapacity = caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader)buffer_header);
                    int next_read = eventcapacity * eventsize;
                    buffer_header = (char *)realloc(buffer_header, 28+next_read);
                    istreamf.read(buffer_header+28,next_read);
                    packetContainerT = caerEventPacketContainerAllocate(1);
                    caerEventPacketContainerSetEventPacket(packetContainerT, 0, (caerEventPacketHeader)buffer_header);
                    caerEventPacketContainerSetEventPacketsNumber(packetContainerT, 1);
                }
                if (packetContainerT != NULL){
                    // Backpressure: wait if queue is full
                    constexpr size_t MAX_QUEUE = 64;
                    while (container.size() >= MAX_QUEUE &&
                           isThreadRunning() && !liveInput && !doChangePath && !paused) {
                        unlock();
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        lock();
                    }
                    container.push_back(packetContainerT);
                }
                if(doChangePath){
                    filename_to_open = path;
                    ofLog(OF_LOG_NOTICE, "Filename "+filename_to_open);
                    // Check if new file is AEDAT4
                    std::string ext3;
                    auto dp3 = filename_to_open.rfind('.');
                    if (dp3 != std::string::npos) ext3 = filename_to_open.substr(dp3);
                    if (ext3 == ".aedat4") {
                        istreamf.close();
                        fileFormat = AedatFormat::AEDAT4;
                        doChangePath = false;
                        fileInputReady = false;
                        goto SELECTFORMAT;
                    }
                    istreamf.open(filename_to_open.c_str(),ios::binary|ios::in);
                    header_skipped = false;
                    doChangePath = false;
                    fileIndexReady = false;
                    resetTimingFlag.store(true);
                    goto HEADERPARSE;
                }
                unlock();
                nanosleep((const struct timespec[]){{0, 5000L}}, NULL);
            }
            } // end AEDAT3.1 else
        }
    }

    caerDeviceHandle camera_handle;
    vector<caerEventPacketContainer> container;
    caerEventPacketContainer packetContainerT;

    bool apsStatus, apsStatusLocal;
    bool dvsStatus, dvsStatusLocal;
    bool imuStatus, imuStatusLocal;
    bool fileInputLocal;
    bool extInputStatus, extInputStatusLocal;
    bool resetTsStatus;

    int sizeX, sizeY;
    bool deviceReady;
    int chipId;
    bool liveInput;

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

    string line;
    string filename_to_open;

    // AEDAT4 file reading
    std::unique_ptr<dv::io::MonoCameraRecording> aedat4Reader;
    AedatFormat fileFormat = AedatFormat::UNKNOWN;
    std::atomic<bool> resetTimingFlag{false};
};

class ofxDVS {
public:
    ofxDVS();
    ~ofxDVS();

    // Methods
    void setup();
    void update();
    void draw();
    void drawFixed();

    // Multi-window support
    void setupCore();      // Camera, threads, buffers, models — no GUI
    void setupGUI();       // All panel creation (call from control window context)
    void drawViewer();     // Visualization only (spikes, images, overlays, labels)
    void drawControls();   // f1 panel (other panels auto-draw via ofEvents)
    void updateGUI();      // f1->update(), text widget refreshes
    void drawSpikes();
    void updateMeshSpikes();
    void drawFrames();
    void drawImu6();
    void initSpikeColors();
    void loopColor();
    void exit();
    bool organizeData(caerEventPacketContainer packetContainer);
    void changeAps();
    void changeDvs();
    void changeImu();
    void changeStats();
    void drawImageGenerator();
    const char * chipIDToName(int16_t chipID, bool withEndSlash);
    static int packetsFirstTimestampThenTypeCmp(const void *a, const void *b);
    void changeRecordingStatus();
    void changeRecordingStatusDb();
    void openRecordingFile();
    void openRecordingFileDb();
    void changeLoadFile();
    bool orginal;
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
    void setPlaybackSpeed(float sliderPos);
    float getPlaybackSpeed();
    void resetPlaybackTiming();
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
    int spkOnR[8], spkOnG[8], spkOnB[8], spkOnA;
    int spkOffR[8], spkOffG[8], spkOffB[8], spkOffA;
    int paletteSpike;
    int maxContainerQueued;
    float fsint;
    bool liveInput;
    bool doDrawSpikes;

    // thread usb
    usbThread thread;
    bool apsStatus, dvsStatus, imuStatus, statsStatus;

    // size
    int sizeX, sizeY, chipId;

    // Image Generator
    ofImage imageGenerator;
    float** spikeFeatures;
    float** surfaceMapLastTs;
    bool rectifyPolarities;
    float numSpikes;
    int counterSpikes;
    bool drawImageGen;
    float decaySpikeFeatures;
    bool newImageGen;

    // Mean Rate
    ofImage meanRateImage;
    float** frequencyMap;
    float** spikeCountMap;
    bool startedMeas;
    float measureMinTime;
    float measureStartedAt;

    // BA filter
    float BAdeltaT;

    // File system
    int isRecording;
    string path;
    bool doChangePath;
    bool header_skipped;
    bool paused;

    // Playback speed
    float playbackSpeed_ = 1.0f;

    // timing information
    long current, started;
    bool isStarted;
    long microseconds, seconds, minutes, hours;
    char timeString[256];

    float imuTemp;

    // mesh
    long tmp, m, nus;

    bool drawDistanceMesh;
    string chipName;
    bool doDrawImu6;

    // cam rotation/translation
    ofQuaternion rotationCam;
    ofVec3f translationCam;
    ofVec3f cameraPos;

    // GUI
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

    int numPaused, numPausedRec;

    // Tracker DVS
    void createRectangularClusterTracker();
    void enableTracker(bool enabled);
    std::unique_ptr<RectangularClusterTracker> rectangularClusterTracker;
    bool rectangularClusterTrackerEnabled = false;
    RectangularClusterTracker::Config rectangularClusterTrackerConfig;
    void setEnabledDvsSensorGuiControls(bool new_state);
    void setEnabledDvsSensorConfigGuiControls(bool new_state);
    std::unique_ptr<ofxDatGui> tracker_panel;
    void mousePressed(int x, int y, int button);


    // Visualisation primitives
    ofTexture       next_frame;
    ofTexture       next_polarities;
    ofPixels        next_polarities_pixbuf;
    ofMesh          next_polarities_3d;
    ofEasyCam       camera;
    ofRectangle     frame_viewport;
    ofRectangle     polarities_viewport;
    ofRectangle     cam_viewport;
    void updateViewports();

    // --- Filter enable flags ---
    bool hotPixelFilterEnabled_ = true;
    bool baFilterEnabled_ = true;

    // Hot pixel suppression — tunable parameters (GUI-accessible)
    int hot_refrac_us = 200;
    int hot_rate_window_us = 100000;   // 100 ms
    int hot_rate_threshold = 500;

    void recalibrateHotPixels();

    // --- Neural network pipelines ---
    bool nnEnabled = false;
    bool tsdtEnabled = false;
    bool tpdvsGestureEnabled = false;

    dvs::YoloPipeline  yolo_pipeline;
    dvs::TsdtPipeline  tsdt_pipeline;
    dvs::TsdtPipeline  tpdvs_gesture_pipeline;

    // Async inference workers
    dvs::InferenceWorker<std::vector<dvs::YoloDet>> yolo_worker;
    dvs::InferenceWorker<std::pair<int,float>>      tsdt_worker;

    // NN / YOLO panel
    std::unique_ptr<ofxDatGui> nn_panel;

    // Optical Flow panel
    std::unique_ptr<ofxDatGui> optflow_panel;

    // Video recording — public for GUI handler access
    float videoRecFps_ = 30.0f;
    ofxFFmpegRecorder videoRecorder_;
    bool              videoRecPaused_ = false;
    bool              videoRecPending_ = false;   // deferred start (from GUI thread)
    std::string       videoRecPath_;
    int      rec_w_ = 0, rec_h_ = 0;
    void     startVideoRecording_();
    void     stopVideoRecording_();

    // Event-based optical flow (local plane fitting on SAE)
    bool  drawOptFlow     = false;
    float optFlowDecay    = 0.95f;
    int   optFlowRadius   = 3;          // neighborhood half-size (7x7)
    int   optFlowDt_us    = 50000;      // 50 ms temporal window
    float optFlowMaxSpeed = 200.0f;     // display clamp (px/s)

    // Event-driven reconstruction — GUI-accessible config
    bool  drawRecon   = false;
    float reconDecay  = 0.97f;
    float reconContrib = 0.15f;
    int   reconSpread = 2;

    // Overlays
    void drawRectangularClusterTracker();

private:
    // Hot pixel suppression — refractory (internal storage)
    std::vector<int64_t> last_ts_map_;

    // Hot pixel suppression — rate-based filter (internal storage)
    std::vector<uint16_t> hot_rate_count_;
    int64_t hot_rate_window_start_ = 0;

    // Hot pixel suppression — startup calibration
    std::vector<uint32_t> hot_calib_count_;
    std::vector<bool> hot_pixel_mask_;
    float hot_calib_duration_s = 3.0f;
    float hot_calib_sigma = 5.0f;
    bool hot_calib_done_ = false;
    int64_t hot_calib_start_ts_ = 0;
    bool hot_calib_started_ = false;

    void applyHotPixelFilter_();
    void finalizeCalibration_();

    // Shutdown guard
    bool exited_ = false;

    // Multi-window mode flag
    bool splitGuiMode_ = false;

    // Point shader
    ofShader pointShader;
    float pointSizePx = 8.0f;

    // Event-driven reconstruction (CPU per-pixel decay) — internal storage
    std::vector<float> reconMap_;   // per-pixel intensity, -1.0 to +1.0
    ofImage reconImage_;            // color output (OF_IMAGE_COLOR)

    // AEDAT4 recording
    std::unique_ptr<dv::io::MonoCameraWriter> aedat4Writer_;

    // Playback timing
    int64_t fileTimeOrigin_    = 0;
    int64_t wallTimeOrigin_    = 0;
    int64_t fileTimePaused_    = 0;
    bool    timingInitialized_ = false;
    float   speedSliderPos_    = 0.0f;   // log10 position: -1..+2

    // Speed display widget
    ofxDatGuiTextInput* mySpeedDisplay = nullptr;

    // Event-based optical flow (SAE + flow maps, private storage)
    std::vector<int64_t> saeTimestamp_;
    std::vector<float>   flowX_, flowY_;
    ofImage              flowImage_;

    // Packet queue management
    std::deque<caerEventPacketContainer> backlog_;
    size_t backlog_max_ = 15;

    // MP4 video recording (internal)
    ofImage  rec_grab_;
    bool     videoRecording_ = false;
};


#endif /* ofxDVS_hpp */
