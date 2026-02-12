//
//  ofxDVS.cpp
//  ofxDVS
//
//  Created by Federico Corradi on 19.05.17.
//  updated since then till 2024
//
//

#include "ofxDVS.hpp"
#include "dvs_gui.hpp"

#include <cmath>
#include <deque>

using namespace std::placeholders;
static constexpr float VIEW_SCALE = 6.0f / 7;

#include <algorithm>
#include <tuple>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cfloat>
#include <ctime>


//--------------------------------------------------------------
ofxDVS::ofxDVS() {

    doChangePath = false;
    header_skipped = false;

    drawGui = true;
}

//--------------------------------------------------------------
void ofxDVS::setup() {

    //thread_alpha.startThread();   // start usb thread
    thread.startThread();   // start usb thread

    // default behaviour is to start live mode
    // after 3 seconds of non finding the device we start in file mode
    ofResetElapsedTimeCounter();
    uint64_t t0 = ofGetElapsedTimeMicros();

    // get camera size after ready
    LOCK_CHECK:
    nanosleep((const struct timespec[]){{0, 50000000L}}, NULL);
    thread.lock();
    if(thread.deviceReady!=true && thread.fileInputReady!=true){
    	thread.unlock();
        uint64_t t1 = ofGetElapsedTimeMicros();
        if( (t1-t0) > 50000){
            ofLog(OF_LOG_NOTICE, "starting in file mode.");
            ofFileDialogResult result = ofSystemLoadDialog("Load aedat3.1 file");
            if(result.bSuccess) {
                path = result.getPath();
                // load your file at `path`
                changePath();
            }
        }
    	goto LOCK_CHECK;
    }

    // viewer started in live or file mode
	sizeX = thread.sizeX;
	sizeY = thread.sizeY;
    chipId = thread.chipId;
    chipName = chipIDToName(chipId, false);
	thread.unlock();

	fsint = 2;

	// init framebuffer
    ofSetVerticalSync(true);
    ofSetBackgroundColor(255);

    // allocate fbo , mesh and imagePol
    fbo.allocate(sizeX, sizeY, GL_RGBA32F);
    tex = &fbo.getTexture();
    mesh.setMode(OF_PRIMITIVE_POINTS);

    // shader for points
    if (ofIsGLProgrammableRenderer()) {
        glEnable(GL_PROGRAM_POINT_SIZE);
        const char* vsrc = R"(
            #version 150
            uniform mat4 modelViewProjectionMatrix;
            uniform float uPointSize;
            in vec4 position;
            in vec4 color;
            out vec4 vColor;
            void main() {
                vColor = color;
                gl_Position = modelViewProjectionMatrix * position;
                gl_PointSize = uPointSize;
            }
        )";
        const char* fsrc = R"(
            #version 150
            in vec4 vColor;
            out vec4 outputColor;
            void main() {
                outputColor = vColor;
            }
        )";
        pointShader.setupShaderFromSource(GL_VERTEX_SHADER,   vsrc);
        pointShader.setupShaderFromSource(GL_FRAGMENT_SHADER, fsrc);
        pointShader.bindDefaults();
        pointShader.linkProgram();
    } else {
        glDisable(GL_POINT_SMOOTH);
        glPointSize(pointSizePx);
    }

    // imagePol
    imagePolarity.allocate(sizeX, sizeY, OF_IMAGE_COLOR);
    newImagePol = false;

    // init spike colors
    initSpikeColors();

    // start the thread
    initThreadVariables();

    // init spikefeature/imagegenerator
    initImageGenerator();

    // init baFilterState
    initBAfilter();

    // init alpha map
    initVisualizerMap();

    // reset timestamp
    ofResetElapsedTimeCounter();
    ofxLastTs = 0;
    targetSpeed = 0.1; // real_time
    paused = false;
    started = 0;
    isStarted = false;
    microseconds = 0;
    seconds = 0;
    minutes = 0;
    hours = 0;
    doDrawSpikes = true;
    imuTemp = 35;

    // mesh
    tmp = 0;
    m = 0;
    nus = 10000;

    drawDistanceMesh = false;
    doDrawImu6 = false;

    //
    //GUI
    int x = 0;
    int y = 0;
    ofSetWindowPosition(0, 0);
    f1 = new ofxDatGuiFolder("Control", ofColor::fromHex(0xFFD00B));
    f1->addBreak();
    f1->addFRM();
    f1->addBreak();
    f1->addSlider("1/speed", 0, 2, targetSpeed);
    myTextTimer = f1->addTextInput("TIME", timeString);
    myTempReader = f1->addTextInput("IMU TEMPERATURE", to_string((int)(imuTemp)));
    f1->addToggle("APS", true);
    f1->addBreak();
    f1->addToggle("DVS", true);
    f1->addBreak();
    f1->addToggle("IMU", true);
    f1->addBreak();
    f1->addMatrix("DVS Color", 7, true);
    f1->addBreak();
    f1->addButton("Clear");
	f1->addBreak();
    f1->addButton("Pause");
	f1->addBreak();
	f1->addToggle("Reset Timestamp", false);
	f1->addBreak();
    f1->addToggle("Ext Input Trigger", false);
    f1->addBreak();
    f1->addButton("Start Recording");
	f1->addBreak();
    f1->addButton("Load Recording");
	f1->addBreak();
    f1->addButton("Live");
	f1->addBreak();
	f1->addToggle("Draw IMU", false);
    f1->addMatrix("3D Time", 4, true);
    f1->addToggle("Pointer", false);
    f1->addToggle("Raw Spikes", true);
    f1->addToggle("DVS Image Gen", false);
    f1->addSlider("Refractory (us)", 0, 5000, hot_refrac_us);
    f1->addSlider("Hot Rate Window (ms)", 10, 1000, hot_rate_window_us / 1000);
    f1->addSlider("Hot Rate Threshold", 10, 5000, hot_rate_threshold);
    f1->addButton("Recalibrate Hot Pixels");
    f1->addSlider("BA Filter dt", 1, 100000, BAdeltaT);
    f1->addSlider("DVS Integration", 1, 100, fsint);
    f1->addSlider("DVS Image Gen", 1, 20000, numSpikes);
    f1->addToggle("ENABLE TRACKER", false);
    f1->addToggle("ENABLE NEURAL NETS", false);

    f1->setPosition(x, y);
    f1->expand();
    f1->onButtonEvent(this, &ofxDVS::onButtonEvent);
    f1->onToggleEvent(this, &ofxDVS::onToggleEvent);
    f1->onSliderEvent(this, &ofxDVS::onSliderEvent);
    f1->onMatrixEvent(this, &ofxDVS::onMatrixEvent);
    f1->onTextInputEvent(this, &ofxDVS::onTextInputEvent);

    numPaused = 0;
    numPausedRec = 0;

    // --- NN / YOLO + TSDT panel (created by dvs::gui helpers) ---
    nn_panel = dvs::gui::createNNPanel(this);

    // --- Tracker panel ---
    tracker_panel = dvs::gui::createTrackerPanel(this);

    /* 2d visualisation primitives setup */
    this->next_polarities_pixbuf.allocate(this->sizeX, this->sizeY, OF_IMAGE_COLOR);
    this->next_polarities.allocate (this->next_polarities_pixbuf);
    this->next_frame.allocate(this->next_polarities_pixbuf);
    updateViewports();

    /* 3d visualisation primitives setup */
    this->next_polarities_3d.setMode(OF_PRIMITIVE_POINTS);
    this->next_polarities_3d.enableColors();

    ofEnableDepthTest();
    ofDisableDepthTest();

    glPointSize(1);

    // --- Load YOLO model via pipeline ---
    try {
        yolo_pipeline.loadModel(ofToDataPath("ReYOLOv8m_PEDRO_352x288.onnx", true));
    } catch (const std::exception& e) {
        ofLogError() << "Failed to load YOLO: " << e.what();
    }

    // --- Load TSDT model via pipeline ---
    try {
        tsdt_pipeline.loadModel(ofToDataPath("spikevision_822128128_fixed.onnx", true));
        tsdt_pipeline.selfTest();
        if (ofFile::doesFileExist(ofToDataPath("tsdt_input_fp32.bin", true))) {
            tsdt_pipeline.debugFromFile(ofToDataPath("tsdt_input_fp32.bin", true));
        }
    } catch (const std::exception& e) {
        ofLogError() << "Failed to load TSDT: " << e.what();
    }

    // Start async YOLO inference worker
    yolo_worker.start();

    // hot pixels
    const size_t npix = this->sizeX * this->sizeY;
    last_ts_map_.assign(npix, 0);
    hot_rate_count_.assign(npix, 0);
    hot_calib_count_.assign(npix, 0);
    hot_pixel_mask_.assign(npix, false);
    hot_calib_done_ = false;
    hot_calib_started_ = false;
}


void ofxDVS::updateViewports()
{
    this->frame_viewport = ofRectangle(20, 20, this->next_frame.getWidth()* VIEW_SCALE, this->next_frame.getHeight() * VIEW_SCALE);
    this->polarities_viewport = ofRectangle(350, 20, this->next_polarities.getWidth()* VIEW_SCALE, this->next_polarities.getHeight()* VIEW_SCALE);
    this->cam_viewport = ofRectangle(700.0, 20, this->sizeX, this->sizeY);
}

//----------------------------------------------------------------------------
void ofxDVS::createRectangularClusterTracker() {
    if (rectangularClusterTrackerEnabled) {
        this->rectangularClusterTracker =
            std::make_unique<RectangularClusterTracker>(
                this->rectangularClusterTrackerConfig,
                this->sizeX, this->sizeY);
    }
}

//----------------------------------------------------------------------------
void ofxDVS::enableTracker(bool enabled) {
    rectangularClusterTrackerEnabled = enabled;
    if (enabled) {
        if (!rectangularClusterTracker) {
            createRectangularClusterTracker();
        }
    } else {
        rectangularClusterTracker.reset();
    }
}


//--------------------------------------------------------------
void ofxDVS::setPointer(bool i){
	drawDistanceMesh = i;
}

//--------------------------------------------------------------
void ofxDVS::setDrawImu(bool i){
	doDrawImu6 = i;
}

//--------------------------------------------------------------
void ofxDVS::initThreadVariables(){
    apsStatus = true;       // enable aps
    dvsStatus = true;       // enable dvs
    imuStatus = true;       // enable imu
    maxContainerQueued = 1000; // at most accumulates 100 packaetcontainers before dropping
    packetContainer = NULL;
    isRecording = false;
}

//--------------------------------------------------------------
void ofxDVS::tryLive(){
    thread.lock();
    // free al memory
    if(thread.fileInput){
        // clean memory
        vector<polarity>().swap(packetsPolarity);
        packetsPolarity.clear();
        packetsPolarity.shrink_to_fit();
        vector<frame>().swap(packetsFrames);
        packetsFrames.clear();
        packetsFrames.shrink_to_fit();
        vector<imu6>().swap(packetsImu6);
        packetsImu6.clear();
        packetsImu6.shrink_to_fit();
        header_skipped  = false;
        thread.fileInput = false;
        thread.header_skipped = header_skipped;
        for(size_t i=0; i<thread.container.size(); i++){
            packetContainer = thread.container.back();
            thread.container.pop_back();
            caerEventPacketContainerFree(packetContainer);
            thread.container.clear();
            thread.container.shrink_to_fit();
        }
    }
    thread.header_skipped  = true;
    thread.fileInput = false;
    thread.liveInput = true;
    liveInput = thread.liveInput;
    thread.unlock();
}

//--------------------------------------------------------------
void ofxDVS::changePath(){
    thread.lock();
    thread.doChangePath = true;
    ofLog(OF_LOG_NOTICE, path);
    // update file path
    thread.path = path;
    // clean memory
    vector<polarity>().swap(packetsPolarity);
    packetsPolarity.clear();
    packetsPolarity.shrink_to_fit();
    vector<frame>().swap(packetsFrames);
    packetsFrames.clear();
    packetsFrames.shrink_to_fit();
    vector<imu6>().swap(packetsImu6);
    packetsImu6.clear();
    packetsImu6.shrink_to_fit();
    thread.istreamf.close();
    thread.header_skipped  = false;
    thread.fileInput = true;
    thread.header_skipped = header_skipped;
    for(size_t i=0; i<thread.container.size(); i++){
        packetContainer = thread.container.back();
        thread.container.pop_back();
        caerEventPacketContainerFree(packetContainer);
        thread.container.clear();
        thread.container.shrink_to_fit();
    }
    liveInput = false;
    thread.unlock();
}

//--------------------------------------------------------------
void ofxDVS::openRecordingFileDb(){
    // File Recording Output
    path = getUserHomeDir();
    if(orginal){
        path = path+"/dataset/";
    }else{
        path = path+"/dataset/selected/";
    }
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    char buffer [80];
    strftime (buffer,80,"ofxDVS_%Y-%m-%d-%I_%M_%S.aedat",now);
    string filename = path + "/" + buffer;
    myFile.open(filename, ios::out | ios::binary);
    writeHeaderFile();
}

//--------------------------------------------------------------
void ofxDVS::openRecordingFile(){
    // File Recording Output
    path = getUserHomeDir();
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    char buffer [80];
    strftime (buffer,80,"ofxDVS_%Y-%m-%d-%I_%M_%S.aedat",now);
    string filename = path + "/" + buffer;
    myFile.open(filename, ios::out | ios::binary);
    writeHeaderFile();
}

//--------------------------------------------------------------
void ofxDVS::writeHeaderFile(){
    // Time
    struct tm currentTime;
    time_t currentTimeEpoch = time(NULL);
    localtime_r(&currentTimeEpoch, &currentTime);
    size_t currentTimeStringLength = 44;
    char currentTimeString[currentTimeStringLength + 1]; // + 1 for terminating NUL byte.
    strftime(currentTimeString, currentTimeStringLength + 1, "#Start-Time: %Y-%m-%d %H:%M:%S (TZ%z)\r\n", &currentTime);

    // Select Chip Id
    size_t sourceStringLength = (size_t) snprintf(NULL, 0, "#Source 0: %s\r\n",chipIDToName(chipId, false));
    char sourceString[sourceStringLength + 1];
    snprintf(sourceString, sourceStringLength + 1, "#Source 0: %s\r\n",chipIDToName(chipId, false));
    sourceString[sourceStringLength] = '\0';
    // Write Header
    myFile << "#!AER-DAT3.1\r\n";
    myFile << "#Format: RAW\r\n";
    myFile << sourceString;
    myFile << currentTimeString;
    myFile << "#!END-HEADER\r\n";
}

//--------------------------------------------------------------
void ofxDVS::changeRecordingStatusDb(){
    apsStatus = false; // no APS
    dvsStatus = true; // yes DVS
    imuStatus = true; // yes IMU
    if(isRecording){
        isRecording = false;
        //close file
        myFile.close();
        ofLog(OF_LOG_NOTICE, "Stop recording\n");
    }else{
        //open file and write header
        resetTs();
        openRecordingFileDb();
        isRecording = true;
        ofLog(OF_LOG_NOTICE, "Start recording\n");

    }
}


//--------------------------------------------------------------
void ofxDVS::changeRecordingStatus(){
    if(isRecording){
        isRecording = false;
        //close file
        myFile.close();
        ofLog(OF_LOG_NOTICE, "Stop recording\n");
    }else{
        //open file and write header
		resetTs();
        openRecordingFile();
        isRecording = true;
        ofLog(OF_LOG_NOTICE, "Start recording\n");

    }
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
    spkOnG[2] = 0;
    spkOnB[2] = 0;
    spkOffR[2] = 0;
    spkOffG[2] = 255;
    spkOffB[2] = 255;
    spkOnR[3] = 0;
    spkOnG[3] = 255;
    spkOnB[3] = 125;
    spkOffR[3] = 0;
    spkOffG[3] = 255;
    spkOffB[3] = 125;
    spkOnR[4] = 0;
    spkOnG[4] = 125;
    spkOnB[4] = 255;
    spkOffR[4] = 125;
    spkOffG[4] = 255;
    spkOffB[4] = 0;
    spkOnR[5] = 0;
    spkOnG[5] = 150;
    spkOnB[5] = 255;
    spkOffR[5] = 255;
    spkOffG[5] = 150;
    spkOffB[5] = 150;
    spkOnR[6] = 0;
    spkOnG[6] = 150;
    spkOnB[6] = 255;
    spkOffR[6] = 150;
    spkOffG[6] = 255;
    spkOffB[6] = 0;
    spkOnR[7] = 150;
    spkOnG[7] = 0;
    spkOnB[7] = 255;
    spkOffR[7] = 150;
    spkOffG[7] = 0;
    spkOffB[7] = 150;

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
void ofxDVS::clearDraw(){

    int clearCol = 0;
    if(paletteSpike == 0){
        clearCol = 0;
    }else if(paletteSpike == 1){
        clearCol = 127;
    }else if(paletteSpike == 2){
        clearCol = 150;
    }else if(paletteSpike == 3){
        clearCol = 0;
    }

	// make a black frame
	frame nuPackFrames;
	nuPackFrames.exposureStart = 0;
	nuPackFrames.exposureEnd = 0;
	nuPackFrames.lenghtX = sizeX;
	nuPackFrames.lenghtY = sizeY;
	nuPackFrames.positionX = 0;
	nuPackFrames.positionY = 0;
	nuPackFrames.singleFrame.allocate(nuPackFrames.lenghtX, nuPackFrames.lenghtY, OF_IMAGE_COLOR);
	for (int32_t y = 0; y < nuPackFrames.lenghtY; y++) {
		for (int32_t x = 0; x < nuPackFrames.lenghtX; x++) {
			ofColor color= ofColor(clearCol,clearCol,clearCol);
			nuPackFrames.singleFrame.setColor(x, y, color);
		}
	}
    nuPackFrames.valid = true;
    nuPackFrames.frameEnd = 0;
    nuPackFrames.frameStart = 0;
    packetsFrames.push_back(nuPackFrames);

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
            continue; // Skip if nothing there.
        }

        int type = caerEventPacketHeaderGetEventType(packetHeader);
        long lastTs = 0;

        if (type == IMU6_EVENT && imuStatus) {

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
            nuPackImu6.temperature = caerIMU6EventGetTemp(caerIMU6IteratorElement);
            nuPackImu6.timestamp = caerIMU6EventGetTimestamp(caerIMU6IteratorElement);
            nuPackImu6.valid = true ;
            packetsImu6.push_back(nuPackImu6);

            if(nuPackImu6.timestamp > lastTs){
                lastTs = nuPackImu6.timestamp;
            }

            CAER_IMU6_ITERATOR_VALID_END

            // keep track of IMU temp
            imuTemp = nuPackImu6.temperature;

        }
        if (type == POLARITY_EVENT  && dvsStatus) {

            packetsPolarity.clear();
            packetsPolarity.shrink_to_fit();

            caerPolarityEventPacket polarity = (caerPolarityEventPacket) packetHeader;

            CAER_POLARITY_ITERATOR_VALID_START(polarity)
            nuPack.timestamp = caerPolarityEventGetTimestamp64(caerPolarityIteratorElement, polarity);

            nuPack.pos.x = caerPolarityEventGetX(caerPolarityIteratorElement);
            nuPack.pos.y = caerPolarityEventGetY(caerPolarityIteratorElement);

            nuPack.pol = caerPolarityEventGetPolarity(caerPolarityIteratorElement);

            nuPack.valid = true;
            packetsPolarity.push_back(nuPack);

            if(nuPack.timestamp > lastTs){
                lastTs = nuPack.timestamp;
            }

            CAER_POLARITY_ITERATOR_VALID_END
        }
        if (type == FRAME_EVENT && apsStatus){

            packetsFrames.clear();
            packetsFrames.shrink_to_fit();

            caerFrameEventPacket frame = (caerFrameEventPacket) packetHeader;

            CAER_FRAME_ITERATOR_VALID_START(frame)
            nuPackFrames.exposureStart = caerFrameEventGetTSStartOfExposure(caerFrameIteratorElement);
            nuPackFrames.exposureEnd = caerFrameEventGetTSEndOfExposure(caerFrameIteratorElement);
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
            nuPackFrames.valid = true;
            nuPackFrames.frameEnd = caerFrameEventGetTSEndOfExposure(caerFrameIteratorElement);
            nuPackFrames.frameStart = caerFrameEventGetTSStartOfExposure(caerFrameIteratorElement);

            packetsFrames.push_back(nuPackFrames);
            nuPackFrames.singleFrame.clear();
            if(nuPackFrames.frameEnd > lastTs){
                lastTs = nuPackFrames.frameEnd;
            }

            CAER_FRAME_ITERATOR_VALID_END
        }
    }

	//keep track of last commit
    ofxLastTs = ofGetElapsedTimeMicros();
    return(true);

}


//--------------------------------------------------------------
void ofxDVS::update() {

    if(paused == false){
        // Copy data from usbThread
        // 1) take ownership quickly
        std::vector<caerEventPacketContainer> local;
        thread.lock();

        if ((thread.deviceReady || thread.fileInputReady) &&
            (thread.sizeX != sizeX || thread.sizeY != sizeY)) {
            sizeX = thread.sizeX;
            sizeY = thread.sizeY;
            if (rectangularClusterTrackerEnabled) createRectangularClusterTracker();
            updateViewports();
        }

        local.swap(thread.container);   // O(1) swap, clears producer queue
        thread.unlock();

        // 2) process WITHOUT holding the lock
        for (auto &packetContainer : local) {
            bool delpc = false;

            long firstTs   = caerEventPacketContainerGetLowestEventTimestamp (packetContainer);
            long highestTs = caerEventPacketContainerGetHighestEventTimestamp(packetContainer);
            long current_file_dt = highestTs - firstTs;
            long current_ofx_dt  = ofGetElapsedTimeMicros() - ofxLastTs;

            if (targetSpeed <= 0) targetSpeed = 1e-7f;

            if (current_file_dt < (float)current_ofx_dt / targetSpeed) {
                delpc = organizeData(packetContainer);

                long ts = caerEventPacketContainerGetHighestEventTimestamp(packetContainer);
                if (ts != -1) {
                    if (!isStarted || ts < started) { started = ts; isStarted = true; }
                    unsigned long cur = ts - started;
                    microseconds = cur - (minutes * 60) * 1e6 - seconds * 1e6;
                    minutes      = (cur / 60e6);
                    seconds      = (((int)cur % (int)60e6) / 1e6);
                    hours        = 0;
                    sprintf(timeString, " %02lu:%02lu:%02lu:%04lu", hours, minutes, seconds, microseconds);
                } else {
                    sprintf(timeString, "%02u", 0u);
                }

                if (isRecording) {
                    size_t n = (size_t)caerEventPacketContainerGetEventPacketsNumber(packetContainer);
                    qsort(packetContainer->eventPackets, n, sizeof(caerEventPacketHeader),
                        &packetsFirstTimestampThenTypeCmp);
                    for (int32_t i = 0; i < (int32_t)n; i++) {
                        auto ph = caerEventPacketContainerGetEventPacket(packetContainer, i);
                        if (!ph) continue;
                        caerEventPacketHeaderSetEventCapacity(ph, caerEventPacketHeaderGetEventNumber(ph));
                        size_t sizePacket = caerEventPacketGetSize(ph);
                        caerEventPacketHeaderSetEventSource(ph, caerEventPacketHeaderGetEventSource(ph));
                        myFile.write((char*)ph, sizePacket);
                    }
                }
            } else {
                // too fast; defer this packet to the next GUI frame
                if (backlog_.size() >= backlog_max_) {
                    caerEventPacketContainerFree(backlog_.front());
                    backlog_.pop_front();
                }
                backlog_.push_back(packetContainer);
            }
        }
    } else {
        // paused: just drop producer data
        thread.lock();
        for (auto &pc : thread.container) caerEventPacketContainerFree(pc);
        thread.container.clear();
        thread.unlock();
    }

    updateBAFilter();
    applyHotPixelFilter_();
    updateImageGenerator();

    // --- FEED RECTANGULAR CLUSTER TRACKER ---
    if (rectangularClusterTrackerEnabled && rectangularClusterTracker) {
        RectangularClusterTracker::PolaritiesQueue inq, outq;
        inq.clear(); outq.clear();

        int64_t latest_ts = 0;
        for (const auto &p : packetsPolarity) {
            if (!p.valid) continue;

            ofxDvsPolarity ev;
            ev.x         = static_cast<int>(p.pos.x);
            ev.y         = static_cast<int>(p.pos.y);
            ev.timestamp = p.timestamp;
            ev.polarity  = p.pol;
            inq.push_back(ev);
            if (ev.timestamp > latest_ts) latest_ts = ev.timestamp;
        }

        rectangularClusterTracker->filter(inq, outq);
        rectangularClusterTracker->updateClusterList(latest_ts);
    }

    // ---- TSDT: push events and run inference (synchronous) ----
    if (!packetsPolarity.empty()) {
        tsdt_pipeline.pushEvents(packetsPolarity, sizeX, sizeY);

        if (tsdtEnabled && tsdt_pipeline.isLoaded()) {
            tsdt_pipeline.infer(sizeX, sizeY);
        }
    }

    //GUI
    f1->update();
    myTextTimer->setText(timeString);
    myTempReader->setText(to_string((int)(imuTemp)));

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
void ofxDVS::changeColor(int i) {
    if(paletteSpike < 7){
        paletteSpike = i;
    }else{
        paletteSpike = 0;
    }
}

void ofxDVS::changeFSInt(float i){
    if(fsint == 0){
        fsint = 0.00001;
    }else{
        fsint = i;
    }
}

//--------------------------------------------------------------
void ofxDVS::changeBAdeltat(float i){
    BAdeltaT = i;
}

//--------------------------------------------------------------
void ofxDVS::drawMouseDistanceToSpikes(){

	if(drawDistanceMesh){
		// Nearest Vertex
		int n = mesh.getNumVertices();
		float nearestDistance = 0;
		ofVec3f nearestVertex;
		ofVec3f nearestVertexCam;
		int nearestIndex = 0;
		int mouseX = ofGetMouseX();
		int mouseY = ofGetMouseY();
		ofVec2f mouse(mouseX, mouseY);
		for(int i = 0; i < n; i++) {
			ofVec3f thisV = mesh.getVertex(i);
			thisV.x = thisV.x-ofGetWidth()/2;
			thisV.y = thisV.y-ofGetHeight()/2;
			ofVec3f cur = myCam.worldToScreen(thisV);
			ofVec3f camCur = mesh.getVertex(i);
			float distance = cur.distance(mouse);
			if(i == 0 || distance < nearestDistance) {
				nearestDistance = distance;
				nearestVertex = cur;
				nearestVertexCam = camCur;
				nearestIndex = i;
			}
		}

		ofPushMatrix();

		ofNoFill();
		ofSetLineWidth(2);
		ofDrawCircle(nearestVertex, 4);
		ofSetLineWidth(1);

		ofVec2f offset(10, -10);
		ofVec2f origXY = ofVec2f(ofMap(nearestVertexCam.x,0,fbo.getWidth(),0,sizeX),ofMap(nearestVertexCam.y,0,fbo.getHeight(),0,sizeY));
		long zconv;
		if(m > 0){
			zconv = (long)(nearestVertexCam.z)<<m;
		}else{
			zconv = (long)nearestVertexCam.z;
		}
		string infos = "x:" + ofToString(origXY.x) + " y:" + ofToString(origXY.y) + " z: "+ofToString(zconv)+" us";
		ofDrawBitmapStringHighlight(infos, mouse + offset);
		ofPopMatrix();
	}

}

//--------------------------------------------------------------
void ofxDVS::drawFixed() {

    //GUI
    if(drawGui){
        f1->draw();
    }
}

void ofxDVS::drawRectangularClusterTracker()
{
    if (!(rectangularClusterTrackerEnabled && rectangularClusterTracker)) return;

    ofPushStyle();
    ofDisableDepthTest();
    ofNoFill();
    ofSetColor(255, 215, 0);   // gold
    ofSetLineWidth(3.0f);

    ofPushMatrix();
    // chip -> full screen (same mapping as drawSpikes)
    ofScale(ofGetWidth() / (float)sizeX, ofGetHeight() / (float)sizeY);
    // flip Y so (0,0) is top-left like the sensor
    ofScale(1.0f, -1.0f);
    ofTranslate(0.0f, -(float)sizeY);

    rectangularClusterTracker->draw(ofRectangle(0, 0, sizeX, sizeY));

    ofPopMatrix();
    ofPopStyle();
}


//--------------------------------------------------------------
void ofxDVS::draw() {

    myCam.begin();
    ofTranslate(ofPoint(-ofGetWidth()/2,-ofGetHeight()/2));
    drawFrames();
    drawImageGenerator(); // if dvs.drawImageGen
    drawSpikes();         // if dvs.doDrawSpikes
    drawImu6();

    drawRectangularClusterTracker();

    // YOLO detections from async worker
    if (nnEnabled && yolo_worker.hasResult()) {
        yolo_pipeline.detections() = yolo_worker.lastResult();
    }
    yolo_pipeline.drawDetections(sizeX, sizeY);

    // TSDT label
    if (tsdtEnabled) {
        tsdt_pipeline.drawLabel();
    }

    myCam.end();

    // Pointer + GUI
    drawMouseDistanceToSpikes();
    if(drawGui){
        f1->draw();
    }

}



/**
 * Event handler when clicking the mouse
 *
 */
void ofxDVS::mousePressed(int x, int y, int button) {
    if (this->rectangularClusterTracker) {
        auto mapper = [](ofRectangle& r, int x, int y, float& mx, float& my) {
            if (r.inside(x, y)) {
                mx = (x - r.getX()) / VIEW_SCALE;
                my = (y - r.getY()) / VIEW_SCALE;
                return true;
            }
            return false;
        };
        float mx, my;
        if (mapper(this->frame_viewport, x, y, mx, my)) {
            if (button == 0) this->rectangularClusterTracker->setVanishingPoint(mx, my);
            if (button == 2) this->rectangularClusterTracker->resetVanishingPoint();
        }
        if (mapper(this->polarities_viewport, x, y, mx, my)) {
            if (button == 0) this->rectangularClusterTracker->setVanishingPoint(mx, my);
            if (button == 2) this->rectangularClusterTracker->resetVanishingPoint();
        }
    }
}

//--------------------------------------------------------------
void ofxDVS::setDrawSpikes(bool doDraw){
    doDrawSpikes = doDraw;
}

//--------------------------------------------------------------
bool ofxDVS::getDrawSpikes(){
    return doDrawSpikes;
}

//--------------------------------------------------------------
void ofxDVS::setIMU(bool value) {
    thread.lock();
    thread.imuStatus = value;
    thread.unlock();
}


//--------------------------------------------------------------
void ofxDVS::resetTs() {
    thread.lock();
    thread.resetTsStatus = true;
    thread.unlock();
}


//--------------------------------------------------------------
void ofxDVS::setExtInput(bool value) {
    thread.lock();
    thread.extInputStatus = value;
    thread.unlock();
}

//--------------------------------------------------------------
void ofxDVS::setDVS(bool value) {
    thread.lock();
    thread.dvsStatus = value;
    thread.unlock();
}

//--------------------------------------------------------------
void ofxDVS::setAPS(bool value) {
    thread.lock();
    thread.apsStatus = value;
    thread.unlock();
}

//--------------------------------------------------------------
void ofxDVS::changeAps() {
    thread.lock();
    bool current_status = thread.apsStatus;
    if(current_status){
        thread.apsStatus = false;
        apsStatus = false;
        ofLog(OF_LOG_NOTICE,"Aps Disabled\n");
    }else{
        thread.apsStatus = true;
        apsStatus = true;
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
        dvsStatus = false;
        ofLog(OF_LOG_NOTICE,"Dvs Disabled\n");
    }else{
        thread.dvsStatus = true;
        dvsStatus = true;
        ofLog(OF_LOG_NOTICE,"Dvs Enabled\n");
    }
    thread.unlock();
}


//--------------------------------------------------------------
void ofxDVS::changeStats() {
    bool current_status = statsStatus;
    if(current_status){
        statsStatus = false;
        ofLog(OF_LOG_NOTICE,"Stats Disabled\n");
    }else{
        statsStatus = true;
        ofLog(OF_LOG_NOTICE,"Stats Enabled\n");
    }
}


//--------------------------------------------------------------
void ofxDVS::changeImu() {
    thread.lock();
    bool current_status = thread.imuStatus;
    if(current_status){
        thread.imuStatus = false;
        imuStatus = false;
        ofLog(OF_LOG_NOTICE,"Imu Disabled\n");
    }else{
        thread.imuStatus = true;
        imuStatus = true;
        ofLog(OF_LOG_NOTICE,"Imu Enabled\n");
    }
    thread.unlock();
}

//--------------------------------------------------------------
void ofxDVS::updateMeshSpikes(){

        for (int i = 0; i < packetsPolarity.size(); i++) {
            int x =(int)packetsPolarity[i].pos.x;
            int y =(int)packetsPolarity[i].pos.y;
            if(packetsPolarity[i].valid){
                visualizerMap[x][y] += 65;
            }
        }
        for( int i=0; i<sizeX; ++i ) {
            for( int j=0; j<sizeY; ++j ) {
                if(visualizerMap[i][j] != 0){
                    visualizerMap[i][j] -= fsint;
                    if(visualizerMap[i][j] < 0){
                        visualizerMap[i][j] = 0;
                    }
                }
                //clear polarity
                imagePolarity.setColor(i, j, ofColor(0,0,0));
            }
        }

        mesh.clear();
        vector<polarity> packets = getPolarity();
        for(int i=0;i < packets.size();i++) {

            int x = (int)packetsPolarity[i].pos.x;
            int y = (int)packetsPolarity[i].pos.y;
            int alpha = 255;

            long tdiff = 0;
            if( packets[i].timestamp < tmp){
                tmp = packets[i].timestamp;
            }
            if(started == false){
                tdiff = 0;
                tmp = packets[i].timestamp;
                started = true;
            }else{
                tdiff = packets[i].timestamp - tmp;
            }
            if(tdiff > nus){
                mesh.clear();
                tdiff = 0;
                tmp = packets[i].timestamp;
            }
            long timeus = 0;
            if(m == 0){
                timeus = 0;
            }else{
                timeus = tdiff>>m;
            }
            mesh.addVertex(ofVec3f(
                ofMap(packets[i].pos.x, 0, sizeX, 0, ofGetWidth()),
                ofMap(packets[i].pos.y, 0, sizeY, 0, ofGetHeight()),
                timeus));

            mesh.addTexCoord(ofVec2f(packets[i].pos.x,packets[i].pos.y));
            ofColor colSO = ofColor(spkOnR[paletteSpike],spkOnG[paletteSpike],spkOnB[paletteSpike],alpha);
            ofColor colSF = ofColor(spkOffR[paletteSpike],spkOffG[paletteSpike],spkOffB[paletteSpike],alpha);
            ofColor this_pixel;
            if(packets[i].pol){
                mesh.addColor(colSO);
                this_pixel.set(colSO);
            }else{
                mesh.addColor(colSF);
                this_pixel.set(colSF);
            }
            if(imagePolarity.isAllocated()){
                imagePolarity.setColor(int(packets[i].pos.x), int(packets[i].pos.y), ofColor(spkOffR[paletteSpike],spkOffG[paletteSpike],spkOffB[paletteSpike]));
            }else{
                ofLog(OF_LOG_ERROR, "imagePol not allocated");
            }
            newImagePol = true;
        }
        imagePolarity.update();
        mesh.setMode(OF_PRIMITIVE_POINTS);

}

//--------------------------------------------------------------
void ofxDVS::drawSpikes() {

    if(doDrawSpikes){
        for (int i = 0; i < packetsPolarity.size(); i++) {
            int x =(int)packetsPolarity[i].pos.x;
            int y =(int)packetsPolarity[i].pos.y;
            if(packetsPolarity[i].valid){
                visualizerMap[x][y] += 65;
            }
        }
        for( int i=0; i<sizeX; ++i ) {
            for( int j=0; j<sizeY; ++j ) {
                if(visualizerMap[i][j] != 0){
                    visualizerMap[i][j] -= fsint;
                    if(visualizerMap[i][j] < 0){
                        visualizerMap[i][j] = 0;
                    }
                }
                //clear polarity
                imagePolarity.setColor(i, j, ofColor(0,0,0));
            }
        }

        mesh.clear();
        vector<polarity> packets = getPolarity();
        for(int i=0;i < packets.size();i++) {

            int x = (int)packetsPolarity[i].pos.x;
            int y = (int)packetsPolarity[i].pos.y;
            int alpha = 255;

            long tdiff = 0;
            if( packets[i].timestamp < tmp){
                tmp = packets[i].timestamp;
            }
            if(started == false){
                tdiff = 0;
                tmp = packets[i].timestamp;
                started = true;
            }else{
                tdiff = packets[i].timestamp - tmp;
            }
            if(tdiff > nus){
                mesh.clear();
                tdiff = 0;
                tmp = packets[i].timestamp;
            }
            long timeus = 0;
            if(m == 0){
                timeus = 0;
            }else{
                timeus = tdiff>>m;
            }
            mesh.addVertex(ofVec3f(ofMap(packets[i].pos.x,0,sizeX,0,ofGetWidth()),ofMap(packets[i].pos.y,sizeY,0,0,ofGetHeight()), timeus));
            mesh.addTexCoord(ofVec2f(packets[i].pos.x,packets[i].pos.y));
            ofColor colSO = ofColor(spkOnR[paletteSpike],spkOnG[paletteSpike],spkOnB[paletteSpike],alpha);
            ofColor colSF = ofColor(spkOffR[paletteSpike],spkOffG[paletteSpike],spkOffB[paletteSpike],alpha);
            ofColor this_pixel;
            if(packets[i].pol){
                mesh.addColor(colSO);
                this_pixel.set(colSO);
            }else{
                mesh.addColor(colSF);
                this_pixel.set(colSF);
            }
            if(imagePolarity.isAllocated()){
                imagePolarity.setColor(int(packets[i].pos.x), int(packets[i].pos.y), ofColor(spkOffR[paletteSpike],spkOffG[paletteSpike],spkOffB[paletteSpike]));
            }else{
                ofLog(OF_LOG_ERROR, "imagePol not allocated");
            }
            newImagePol = true;
        }
        imagePolarity.update();
        mesh.setMode(OF_PRIMITIVE_POINTS);
        ofPushMatrix();
        if (ofIsGLProgrammableRenderer()) {
            pointShader.begin();
            pointShader.setUniform1f("uPointSize", 4);
            mesh.draw();
            pointShader.end();
        } else {
            glDisable(GL_POINT_SMOOTH);
            glPointSize(4);
            mesh.draw();
        }

        ofPopMatrix();

    }
}

//--------------------------------------------------------------
void ofxDVS::drawFrames() {
    // draw last frame in packets
    for (int i = 0; i < packetsFrames.size(); i++) {
        packetsFrames[i].singleFrame.draw(0,0,ofGetWidth(),ofGetHeight());
    }
}

//--------------------------
void ofxDVS::initBAfilter(){
    baFilterMap=new long*[sizeX];
    for( int i=0; i<sizeX; ++i ) {
        baFilterMap[i] = new long[sizeY];
        for( int j=0; j<sizeY; ++j ) {
            baFilterMap[i][j] = 0;
        }
    }

    BAdeltaT = 3000;
}

//--------------------------
void ofxDVS::initVisualizerMap(){
    visualizerMap=new float*[sizeX];
    for( int i=0; i<sizeX; ++i ) {
        visualizerMap[i] = new float[sizeY];
        for( int j=0; j<sizeY; ++j ) {
            visualizerMap[i][j] = 0.0;
        }
    }
}

//--------------------------
// Background Activity Filter
void ofxDVS::updateBAFilter(){

    for (int i = 0; i < packetsPolarity.size(); i++) {
        ofPoint pos = packetsPolarity[i].pos;

        // get last spike time
        int lastTS = baFilterMap[(int)pos.x][(int)pos.y];
        int ts = packetsPolarity[i].timestamp;

        if (( (ts - lastTS) >= BAdeltaT) || (lastTS == 0)) {
            // Filter out invalid, simply invalid them
            packetsPolarity[i].valid = false;
        }

        // Update neighboring region.
        size_t sizeMaxX = (sizeX - 1);
        size_t sizeMaxY = (sizeY - 1);

        int x = pos.x;
        int y = pos.y;

        if (x > 0) {
            baFilterMap[x - 1][y] = ts;
        }
        if (x < sizeMaxX) {
            baFilterMap[x + 1][y] = ts;
        }

        if (y > 0) {
            baFilterMap[x][y - 1] = ts;
        }
        if (y < sizeMaxY) {
            baFilterMap[x][y + 1] = ts;
        }

        if (x > 0 && y > 0) {
            baFilterMap[x - 1][y - 1] = ts;
        }
        if (x < sizeMaxX && y < sizeMaxY) {
            baFilterMap[x + 1][y + 1] = ts;
        }

        if (x > 0 && y < sizeMaxY) {
            baFilterMap[x - 1][y + 1] = ts;
        }
        if (x < sizeMaxX && y > 0) {
            baFilterMap[x + 1][y - 1] = ts;
        }

    }
}


//--------------------------------------------------------------
void ofxDVS::changeLoadFile() {

    thread.fileInput = !thread.fileInput;
    ofLog(OF_LOG_WARNING,"FileInput mode a");
    if(thread.fileInput){
    	ofLog(OF_LOG_WARNING,"FileInput mode");
        ofFileDialogResult result = ofSystemLoadDialog("Load aedat file");
        if(result.bSuccess) {
            path = result.getPath();
            // load your file at `path`
            changePath();
        }
    }else{
        if(thread.fileInputReady){
            thread.istreamf.close();
            thread.fileInputReady = false;
        }
    }

    // disable video, then go back live
    if(thread.fileInput == false){
        liveInput = true;
        // open video grabber
        // recording files
        isRecording = false;

        thread.fileInputLocal = false;
        thread.fileInputReady = false;
        thread.fileIndexReady = false;

    }
}

//--------------------------------------------------------------
void ofxDVS::loadFile() {
	ofLog(OF_LOG_WARNING,"loadfiles");
    ofFileDialogResult result = ofSystemLoadDialog("Load aedat file");
    if(result.bSuccess) {
        path = result.getPath();
        // load your file at `path`
        changePath();
    }
}

//--------------------------------------------------------------
ofTexture* ofxDVS::getTextureRef() {
    return tex;
}

//--------------------------------------------------------------
ofxDVS::~ofxDVS() {
    exit();
}

void ofxDVS::exit() {
    if (exited_) return;
    exited_ = true;

    ofLogNotice() << "[ofxDVS] exit: stopping workers...";

    // stop inference workers first (they may hold ONNX sessions)
    yolo_worker.stop();
    tsdt_worker.stop();

    ofLogNotice() << "[ofxDVS] exit: stopping USB thread...";

    // signal the USB thread to stop
    thread.stopThread();

    // give the thread time to notice the flag and finish
    for (int i = 0; i < 50; ++i) {
        if (!thread.isThreadRunning()) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // detach to prevent std::terminate if thread is still alive
    try {
        auto& t = thread.getNativeThread();
        if (t.joinable()) {
            t.detach();
        }
    } catch (...) {}

    if (isRecording) {
        myFile.close();
    }

    ofLogNotice() << "[ofxDVS] exit: done";
}

const char * ofxDVS::chipIDToName(int16_t chipID, bool withEndSlash) {
    switch (chipID) {
        case 0:
            return ((withEndSlash) ? ("DAVIS240A/") : ("DAVIS240A"));
            break;

        case 1:
            return ((withEndSlash) ? ("DAVIS240B/") : ("DAVIS240B"));
            break;

        case 2:
            return ((withEndSlash) ? ("DAVIS240C/") : ("DAVIS240C"));
            break;

        case 3:
            return ((withEndSlash) ? ("DAVIS128/") : ("DAVIS128"));
            break;

        case 4:
            return ((withEndSlash) ? ("DAVIS346A/") : ("DAVIS346A"));
            break;

        case 5:
            return ((withEndSlash) ? ("DAVIS346B/") : ("DAVIS346B"));
            break;

        case 6:
            return ((withEndSlash) ? ("DAVIS640/") : ("DAVIS640"));
            break;

        case 7:
            return ((withEndSlash) ? ("DAVISHet640/") : ("DAVISHet640"));
            break;

        case 8:
            return ((withEndSlash) ? ("DAVIS208/") : ("DAVIS208"));
            break;

        case 9:
            return ((withEndSlash) ? ("DAVIS346Cbsi/") : ("DAVIS346Cbsi"));
            break;
    }

    return ((withEndSlash) ? ("Unknown/") : ("Unknown"));
}

// --- Sort packet container by timestamp and type ID
int ofxDVS::packetsFirstTimestampThenTypeCmp(const void *a, const void *b) {
    const caerEventPacketHeader *aa = (const caerEventPacketHeader *)a;
    const caerEventPacketHeader *bb = (const caerEventPacketHeader *)b;

    if(*aa == NULL && *bb == NULL){
        return(0);
    }else if( *aa == NULL && *bb != NULL){
        return(-1);
    }else if( *aa != NULL && *bb == NULL){
        return(1);
    }

    // Sort first by timestamp of the first event.
    int32_t eventTimestampA = caerGenericEventGetTimestamp(caerGenericEventGetEvent(*aa, 0), *aa);
    int32_t eventTimestampB = caerGenericEventGetTimestamp(caerGenericEventGetEvent(*bb, 0), *bb);

    if (eventTimestampA < eventTimestampB) {
        return (-1);
    }
    else if (eventTimestampA > eventTimestampB) {
        return (1);
    }
    else {
        // If equal, further sort by type ID.
        int16_t eventTypeA = caerEventPacketHeaderGetEventType(*aa);
        int16_t eventTypeB = caerEventPacketHeaderGetEventType(*bb);

        if (eventTypeA < eventTypeB) {
            return (-1);
        }
        else if (eventTypeA > eventTypeB) {
            return (1);
        }
        else {
            return (0);
        }
    }
}

string ofxDVS::getUserHomeDir()
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

//
//--------------------------------------------------------------
void ofxDVS::initMeanRate(){
    frequencyMap = new float*[sizeX];
    for(int i = 0; i < sizeX; ++i){
        frequencyMap[i] = new float[sizeY];
    }
    for(int i = 0; i < sizeX; ++i){
        for(int j = 0; j < sizeY; ++j){
            frequencyMap[i][j] = 0.0;
        }
    }
    spikeCountMap = new float*[sizeX];
    for(int i = 0; i < sizeX; ++i){
        spikeCountMap[i] = new float[sizeY];
    }
    for(int i = 0; i < sizeX; ++i){
        for(int j = 0; j < sizeY; ++j){
            spikeCountMap[i][j] = 0.0;
        }
    }
    meanRateImage.allocate(sizeX, sizeY, OF_IMAGE_COLOR_ALPHA);
    startedMeas = false;
}

// Image Generator
//--------------------------------------------------------------
void ofxDVS::initImageGenerator(){
    spikeFeatures = new float*[sizeX];
    for(int i = 0; i < sizeX; ++i){
        spikeFeatures[i] = new float[sizeY];
    }
    for(int i = 0; i < sizeX; ++i){
        for(int j = 0; j < sizeY; ++j){
            spikeFeatures[i][j] = 0.0;
        }
    }
    surfaceMapLastTs = new float*[sizeX];
    for(int i = 0; i < sizeX; ++i){
        surfaceMapLastTs[i] = new float[sizeY];
    }
    for(int i = 0; i < sizeX; ++i){
        for(int j = 0; j < sizeY; ++j){
            surfaceMapLastTs[i][j] = 0.0;
        }
    }
    imageGenerator.allocate(sizeX, sizeY, OF_IMAGE_COLOR);
    rectifyPolarities = false;
    numSpikes = 1500.0;
    counterSpikes = 0;
    drawImageGen = false;
    decaySpikeFeatures = 0.02;
    newImageGen = false;
}

//--------------------------------------------------------------
void ofxDVS::drawImageGenerator() {

    if(drawImageGen){
        imageGenerator.draw(0,0,ofGetWidth(),ofGetHeight());
    }
}

//--------------------------------------------------------------
void ofxDVS::drawImu6() {

	if(doDrawImu6){
		float ap = 0.955;
		double compAngleX, compAngleY,compAngleZ, timer;
		double accXangle , accYangle, accZangle;
		double gyroXrate , gyroYrate, gyroZrate;
		double gyroXAngle, gyroYAngle, gyroZAngle;
		ofPushStyle();
		for(size_t i=0; i < packetsImu6.size(); i++){

			//Complementary filter
			accXangle = (atan2(packetsImu6[i].accel.y, packetsImu6[i].accel.z) * RAD_TO_DEG);
			accYangle = (atan2(packetsImu6[i].accel.x, packetsImu6[i].accel.z) * RAD_TO_DEG);
			accZangle = (atan2(packetsImu6[i].accel.x, packetsImu6[i].accel.y) * RAD_TO_DEG);
			gyroXrate = packetsImu6[i].gyro.x / 16.5;
			gyroYrate = packetsImu6[i].gyro.y / 16.5;
			gyroZrate = packetsImu6[i].gyro.z / 16.5;
			timer = ofGetElapsedTimeMillis();

			gyroXAngle += gyroXrate * (ofGetElapsedTimeMillis() - timer) / 1000;
			gyroYAngle += gyroYrate * (ofGetElapsedTimeMillis() - timer) / 1000;
			gyroZAngle += gyroZrate * (ofGetElapsedTimeMillis() - timer) / 1000;
			compAngleX = ap * (compAngleX + gyroXAngle) + (1 - ap) * accXangle;
			compAngleY = ap * (compAngleY + gyroYAngle) + (1 - ap) * accYangle;
			compAngleZ = ap * (compAngleZ + gyroZAngle) + (1 - ap) * accZangle;

			ofSetColor(ofColor::red);
			ofVec3f arrowHeadPoint = ofVec3f(ofGetHeight()/2,ofGetWidth()/2,0);
			ofVec3f arrowTailPoint = ofVec3f(ofGetHeight()/2+packetsImu6[i].accel.x*3,ofGetWidth()/2+packetsImu6[i].accel.y*3,packetsImu6[i].accel.z*3);
			ofDrawArrow(arrowTailPoint,arrowHeadPoint, 10.0);
			ofSetColor(ofColor::black);

			ofSetColor(ofColor::green);
			arrowHeadPoint = ofVec3f(ofGetHeight()/1.5,ofGetWidth()/2,0);
			arrowTailPoint = ofVec3f(ofGetHeight()/1.5+packetsImu6[i].gyro.x,ofGetWidth()/2+packetsImu6[i].gyro.y,packetsImu6[i].gyro.z);
			ofDrawArrow(arrowTailPoint,arrowHeadPoint, 10.0);
			ofSetColor(ofColor::black);

		}
		ofPopStyle();
	}
}


//--------------------------------------------------------------
void ofxDVS::setDrawImageGen(bool doDraw){
    drawImageGen = doDraw;
}

//--------------------------------------------------------------
bool ofxDVS::getDrawImageGen(){
    return drawImageGen;
}

//--------------------------------------------------------------
void ofxDVS::setImageAccumulatorSpikes(float value){
    numSpikes = value;
}

//--------------------------------------------------------------
void ofxDVS::setImageAccumulatorDecay(float value){
    decaySpikeFeatures = value;
}

//--------------------------------------------------------------
void ofxDVS::set3DTime(int i){
    if(i == 0){
        m = 0;
    }else if(i == 1){
        m = 8;
    }else if(i == 2){
        m = 6;
    }else if(i == 3){
        m = 4;
    }else if(i == 4){
        m = 2;
    }
}

//--------------------------------------------------------------
void ofxDVS::updateImageGenerator(){

    long lastTs;
    for (int i = 0; i < packetsPolarity.size(); i++) {
        // only valid
        if(packetsPolarity[i].valid){
            ofPoint pos = packetsPolarity[i].pos;

            // update surfaceMap
            if(packetsPolarity[i].pol){
                spikeFeatures[(int)pos.x][(int)pos.y] = 1.0;
            }else{
                spikeFeatures[(int)pos.x][(int)pos.y] = 1.0;
            }

            surfaceMapLastTs[(int)pos.x][(int)pos.y] = packetsPolarity[i].timestamp;
            lastTs = packetsPolarity[i].timestamp;
            counterSpikes = counterSpikes+1;
        }
    }

    if(numSpikes <= counterSpikes){

        counterSpikes = 0;
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
            range = numSDevs * sig * (1.0f / 256.0f);
            halfrange = 0;
        }

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
                        f = 255;
                    }else if (f < 0) {
                        f = 0;
                    }
                    ofColor this_pixel;
                    this_pixel.set((int)floor(f), spkOnR[paletteSpike],spkOnG[paletteSpike]);
                    imageGenerator.setColor(col_idx,row_idx,this_pixel);
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

        newImageGen = true;

        // --- YOLO inference via async worker ---
        if (nnEnabled && yolo_pipeline.isLoaded() && newImageGen) {
            // Build VTEI tensor on main thread (fast, uses pipeline pre-allocated buffers)
            auto vtei = yolo_pipeline.buildVTEI(
                packetsPolarity, surfaceMapLastTs,
                imageGenerator.getPixels(), sizeX, sizeY);

            // Submit inference to worker thread (non-blocking; dropped if worker is busy)
            int sw = sizeX, sh = sizeY;
            yolo_worker.submit([this, vtei = std::move(vtei), sw, sh]() -> std::vector<dvs::YoloDet> {
                yolo_pipeline.infer(vtei, sw, sh);
                return yolo_pipeline.detections();
            });
        }
    }
}




//--------------------------------------------------------------
float ofxDVS::getImuTemp(){
    return(imuTemp);
}

//--------------------------------------------------------------
void ofxDVS::changeTargetSpeed(float val){
    targetSpeed = targetSpeed + val;
    ofLog(OF_LOG_NOTICE, "Target speed is now %f", targetSpeed);
}

//--------------------------------------------------------------
void ofxDVS::setTargetSpeed(float val){
	targetSpeed = val;
}

//--------------------------------------------------------------
float ofxDVS::getTargetSpeed(){
    return(targetSpeed);
}

//--------------------------------------------------------------
void ofxDVS::changePause(){
    if(paused){
        paused = false;
        thread.lock();
        thread.paused = paused;
        thread.unlock();
    }else{
        paused = true;
        thread.lock();
        thread.paused = paused;
        thread.unlock();
    }
}

//--------------------------------------------------------------
void ofxDVS::setPause(bool value){
	paused = value;
	thread.lock();
	thread.paused = paused;
	thread.unlock();
}

//--------------------------------------------------------------
ofImage ofxDVS:: getImageGenerator(){

    return imageGenerator;

}



//--------------------------------------------------------------
void ofxDVS::keyPressed(int key){
    if (key == 'c') {
        changeDrawGui();
    }
}

void ofxDVS::changeDrawGui(){
    if(drawGui){
        drawGui = false;
    }else{
        drawGui = true;
    }
}

void ofxDVS::onButtonEvent(ofxDatGuiButtonEvent e)
{
	if(e.target->getLabel() == "Clear"){
		clearDraw();
	}else if( (e.target->getLabel() == "Pause") ||  (e.target->getLabel() == "Start")){
		numPaused++;
		if((numPaused % 2) == 0){
			e.target->setLabel("Pause");
		}else{
			e.target->setLabel("Start");
		}
		changePause();
	}else if( (e.target->getLabel() == "Start Recording") ||  (e.target->getLabel() == "Stop Recording")){
		numPausedRec++;
		if((numPausedRec % 2) == 0){
			e.target->setLabel("Start Recording");
		}else{
			e.target->setLabel("Stop Recording");
		}
		changeRecordingStatus();
	}else if(e.target->getLabel() == "Load Recording"){
		loadFile();
	}else if(e.target->getLabel() == "Live"){
		tryLive();
    }else if ( (e.target->getLabel() == "Enable NN") || (e.target->getLabel() == "Disable NN") ) {
        nnEnabled = !nnEnabled;
        e.target->setLabel(nnEnabled ? "Disable NN" : "Enable NN");
        ofLogNotice() << "NN execution " << (nnEnabled ? "enabled" : "disabled");
    }else if (e.target->getLabel() == "Recalibrate Hot Pixels") {
        const size_t npix = sizeX * sizeY;
        hot_calib_done_ = false;
        hot_calib_started_ = false;
        hot_calib_start_ts_ = 0;
        std::fill(hot_calib_count_.begin(), hot_calib_count_.end(), 0);
        std::fill(hot_pixel_mask_.begin(), hot_pixel_mask_.end(), false);
        ofLogNotice() << "[HotPixel] Calibration restarted";
    }
}


void ofxDVS::onToggleEvent(ofxDatGuiToggleEvent e)
{
    if (e.target->is("ENABLE TRACKER")) {
        auto checked = e.target->getChecked();
        this->tracker_panel->setVisible(checked);
        this->enableTracker(checked);
        this->rectangularClusterTrackerConfig.useVelocity = true;
        this->rectangularClusterTrackerConfig.thresholdVelocityForVisibleCluster = 30.f;

        ofLog(OF_LOG_NOTICE, "Tracker enabled %d", checked);
    }else if(e.target->getLabel() == "APS"){
    	changeAps();
    }else if(e.target->getLabel() == "DVS"){
    	changeDvs();
    }else if(e.target->getLabel() == "IMU"){
    	changeImu();
    }else if (e.target->getLabel() == "DVS Image Gen"){
        setDrawImageGen(e.target->getChecked());
    }else if(e.target->getLabel() == "Raw Spikes"){
        setDrawSpikes(e.target->getChecked());
    }else if(e.target->getLabel() == "Pointer"){
        setPointer(e.target->getChecked());
    }else if(e.target->getLabel() == "Draw IMU"){
        setDrawImu(e.target->getChecked());
    }else if( e.target->getLabel() == "Ext Input Trigger"){
    	setExtInput(e.target->getChecked());
    }else if( e.target->getLabel() == "Reset Timestamp"){
    	resetTs();
    	e.target->setChecked(false); // reset to false
    }else if (e.target->getLabel() == "Draw YOLO") {
        yolo_pipeline.cfg.draw = e.target->getChecked();
    }else if (e.target->is("ENABLE NEURAL NETS")) {
        bool checked = e.target->getChecked();
        // show/hide panel
        if (this->nn_panel) this->nn_panel->setVisible(checked);
        // actually enable/disable inference
        nnEnabled = checked;
        // make sure nothing keeps drawing when disabled
        if (!checked) {
            yolo_pipeline.clearHistory();
        }
        ofLog(OF_LOG_NOTICE, "Neural nets %s", checked ? "enabled" : "disabled");
    }

}

void ofxDVS::onSliderEvent(ofxDatGuiSliderEvent e)
{
    if(e.target->getLabel() == "1/speed"){
        cout << "onSliderEvent speed is : " << e.value << endl;
        setTargetSpeed(e.value);
    }else if(e.target->getLabel() == "DVS Integration"){
        cout << "Integration fsint is : " << e.value << endl;
        changeFSInt(e.value);
    }else if( e.target->getLabel() == "BA Filter dt"){
        cout << "BackGround Filter dt : " << e.value << endl;
        changeBAdeltat(e.value);
    }else if( e.target->getLabel() == "DVS Image Gen"){
        cout << "Accumulation value : " << e.value << endl;
        setImageAccumulatorSpikes(e.value);
    }
    else if (e.target->getLabel() == "YOLO Conf") {
        yolo_pipeline.cfg.conf_thresh = e.value;
    }
    else if (e.target->getLabel() == "VTEI Window (ms)") {
        yolo_pipeline.cfg.vtei_win_ms = e.value;
        ofLogNotice() << "VTEI window set to " << e.value << " ms";
    }else if (e.target->getLabel() == "Refractory (us)") {
        hot_refrac_us = (int)e.value;
    }else if (e.target->getLabel() == "Hot Rate Window (ms)") {
        hot_rate_window_us = (int)e.value * 1000;
        ofLogNotice() << "[HotPixel] Rate window set to " << hot_rate_window_us << " us";
    }else if (e.target->getLabel() == "Hot Rate Threshold") {
        hot_rate_threshold = (int)e.value;
        ofLogNotice() << "[HotPixel] Rate threshold set to " << hot_rate_threshold;
    }

    myCam.reset(); // no mesh turning when using GUI
}

// hot pixel calibration finalization
void ofxDVS::finalizeCalibration_() {
    const size_t npix = sizeX * sizeY;
    // compute mean and stddev of per-pixel event counts
    double sum = 0.0, sum2 = 0.0;
    for (size_t i = 0; i < npix; ++i) {
        double c = hot_calib_count_[i];
        sum  += c;
        sum2 += c * c;
    }
    double mean = sum / npix;
    double var  = (sum2 / npix) - (mean * mean);
    double sd   = (var > 0.0) ? std::sqrt(var) : 0.0;
    double thresh = mean + hot_calib_sigma * sd;

    int nHot = 0;
    for (size_t i = 0; i < npix; ++i) {
        if (hot_calib_count_[i] > thresh) {
            hot_pixel_mask_[i] = true;
            ++nHot;
        }
    }
    hot_calib_done_ = true;
    ofLogNotice() << "[HotPixel] Calibration: found " << nHot
                  << " hot pixels (mean=" << mean << " sd=" << sd
                  << " thresh=" << thresh << ")";
}

// hot pixel filter: calibration + refractory + rate-based
void ofxDVS::applyHotPixelFilter_() {
    const int W = sizeX, H = sizeY;

    for (auto &e : packetsPolarity) {
        if (!e.valid) continue;
        int x = (int)e.pos.x, y = (int)e.pos.y;
        if ((unsigned)x >= (unsigned)W || (unsigned)y >= (unsigned)H) {
            e.valid = false;
            continue;
        }
        int idx = y * W + x;

        // --- calibration phase ---
        if (!hot_calib_done_) {
            if (!hot_calib_started_) {
                hot_calib_started_ = true;
                hot_calib_start_ts_ = e.timestamp;
            }
            hot_calib_count_[idx]++;
            int64_t elapsed = e.timestamp - hot_calib_start_ts_;
            if (elapsed >= (int64_t)(hot_calib_duration_s * 1e6f)) {
                finalizeCalibration_();
            }
        }

        // --- calibration mask ---
        if (hot_calib_done_ && hot_pixel_mask_[idx]) {
            e.valid = false;
            continue;
        }

        // --- refractory check ---
        int64_t last = last_ts_map_[idx];
        if (last != 0 && (e.timestamp - last) < hot_refrac_us) {
            e.valid = false;
            continue;
        }
        last_ts_map_[idx] = e.timestamp;

        // --- rate-based check ---
        // reset window if rolled over
        if (hot_rate_window_start_ == 0) {
            hot_rate_window_start_ = e.timestamp;
        }
        if (e.timestamp - hot_rate_window_start_ >= hot_rate_window_us) {
            std::fill(hot_rate_count_.begin(), hot_rate_count_.end(), 0);
            hot_rate_window_start_ = e.timestamp;
        }
        hot_rate_count_[idx]++;
        if (hot_rate_count_[idx] > hot_rate_threshold) {
            e.valid = false;
        }
    }
}

void ofxDVS::onTextInputEvent(ofxDatGuiTextInputEvent e)
{
    cout << "onTextInputEvent" << endl;
}

void ofxDVS::on2dPadEvent(ofxDatGui2dPadEvent e)
{
    cout << "on2dPadEvent" << endl;
}

void ofxDVS::onColorPickerEvent(ofxDatGuiColorPickerEvent e)
{
    cout << "onColorPickerEvent" << endl;
}

void ofxDVS::onMatrixEvent(ofxDatGuiMatrixEvent e)
{

    if( e.target->getLabel() == "3D Time"){
        e.target->setRadioMode(true);
        for(size_t i = 0; i < 4 ; i++){
            if(e.child == i){
                set3DTime(i);
            }
        }
    }else if(e.target->getLabel() == "DVS Color"){
        e.target->setRadioMode(true);
        for(size_t i = 0; i < 6 ; i++){
            if(e.child == i){
                changeColor(i);
            }
        }
    }
}
