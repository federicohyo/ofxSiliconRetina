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
// setupCore() — everything except GUI panel creation
void ofxDVS::setupCore() {
    splitGuiMode_ = true;

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
            ofFileDialogResult result = ofSystemLoadDialog("Load recording (.aedat / .aedat4)");
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
    ofSetBackgroundColor(0);

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

    // --- Event reconstruction (CPU per-pixel decay) ---
    reconMap_.assign(sizeX * sizeY, 0.0f);
    reconImage_.allocate(sizeX, sizeY, OF_IMAGE_COLOR);

    // --- Event-based optical flow (SAE + local plane fitting) ---
    saeTimestamp_.assign(sizeX * sizeY, 0);
    flowX_.assign(sizeX * sizeY, 0.0f);
    flowY_.assign(sizeX * sizeY, 0.0f);
    flowImage_.allocate(sizeX, sizeY, OF_IMAGE_COLOR);

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
    playbackSpeed_ = 1.0f;
    speedSliderPos_ = 0.0f;
    timingInitialized_ = false;
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

    numPaused = 0;
    numPausedRec = 0;

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
        yolo_pipeline.setupProbes();
    } catch (const std::exception& e) {
        ofLogError() << "Failed to load YOLO: " << e.what();
    }

    // --- Load SNN YOLO model (twl_spike_yolo / yolo8n_gen1) ---
    try {
        snn_yolo_pipeline.loadModel(
            ofToDataPath("yolo8n_gen1_stateful.onnx", true),
            ofToDataPath("yolo8n_gen1_stateful_anchors.npy", true));
    } catch (const std::exception& e) {
        ofLogWarning() << "[SnnYolo] load failed: " << e.what();
    }

    // --- Load TSDT model via pipeline ---
    try {
        tsdt_pipeline.loadModel(ofToDataPath("sew_resnet_dvs_gesture.onnx",true));//tp_gesture_128x128.onnx", true));
        tsdt_pipeline.selfTest();
        if (ofFile::doesFileExist(ofToDataPath("tsdt_input_fp32.bin", true))) {
            tsdt_pipeline.debugFromFile(ofToDataPath("tsdt_input_fp32.bin", true));
        }
    } catch (const std::exception& e) {
        ofLogError() << "Failed to load TSDT: " << e.what();
    }

    // --- Load TPDVSGesture model via pipeline ---
    try {
        tpdvs_gesture_pipeline.cfg.T = 1;
        tpdvs_gesture_pipeline.cfg.inH = 32;
        tpdvs_gesture_pipeline.cfg.inW = 32;
        tpdvs_gesture_pipeline.cfg.time_based_binning = true;
        tpdvs_gesture_pipeline.cfg.bin_window_ms = 75.0f;
        tpdvs_gesture_pipeline.cfg.stateful = true;
        tpdvs_gesture_pipeline.cfg.ema_alpha = 1.0f;  // SNN state handles temporal integration
        tpdvs_gesture_pipeline.cfg.label_y_offset = -80.f;
        tpdvs_gesture_pipeline.cfg.log_tag = "TPDVSGesture";
        tpdvs_gesture_pipeline.loadModel(ofToDataPath("tp_gesture_paper_32x32.onnx", true));
        tpdvs_gesture_pipeline.setupProbes();
    } catch (const std::exception& e) {
        ofLogError() << "Failed to load TPDVSGesture: " << e.what();
    }

    // Start async inference workers
    yolo_worker.start();

    // hot pixels
    const size_t npix = this->sizeX * this->sizeY;
    last_ts_map_.assign(npix, 0);
    hot_rate_count_.assign(npix, 0);
    hot_calib_count_.assign(npix, 0);
    hot_pixel_mask_.assign(npix, false);
    hot_calib_done_ = false;
    hot_calib_started_ = false;

    // Reset camera to default so image generator fills the viewer at startup
    myCam.reset();
}

//--------------------------------------------------------------
// setupGUI() — all GUI panel creation (call from control window context)
void ofxDVS::setupGUI() {
    int x = 0;
    int y = 0;
    f1 = new ofxDatGuiFolder("Control", ofColor::fromHex(0xFFD00B));
    f1->addBreak();
    f1->addFRM();
    f1->addBreak();
    f1->addSlider("Playback Speed", -1, 2, speedSliderPos_);
    mySpeedDisplay = f1->addTextInput("SPEED", "1.0x");
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
    f1->addToggle("HOT PIXEL FILTER", hotPixelFilterEnabled_);
    f1->addToggle("BA FILTER", baFilterEnabled_);
    f1->addSlider("DVS Integration", 1, 100, fsint);
    f1->addSlider("DVS Image Gen", 1, 20000, numSpikes);
    f1->addToggle("ENABLE OPTICAL FLOW", false);
    f1->addToggle("ENABLE TRACKER", false);
    f1->addToggle("ENABLE NEURAL NETS", false);

    f1->setPosition(x, y);
    f1->expand();
    f1->onButtonEvent(this, &ofxDVS::onButtonEvent);
    f1->onToggleEvent(this, &ofxDVS::onToggleEvent);
    f1->onSliderEvent(this, &ofxDVS::onSliderEvent);
    f1->onMatrixEvent(this, &ofxDVS::onMatrixEvent);
    f1->onTextInputEvent(this, &ofxDVS::onTextInputEvent);

    // --- NN / YOLO + TSDT panel (created by dvs::gui helpers) ---
    // (Video Output folder is inside this panel, between Filters and YOLO)
    nn_panel = dvs::gui::createNNPanel(this);

    // --- Optical Flow panel ---
    optflow_panel = dvs::gui::createOptFlowPanel(this);

    // --- Tracker panel ---
    tracker_panel = dvs::gui::createTrackerPanel(this);
}

//--------------------------------------------------------------
// setup() — backward-compatible single-window setup
void ofxDVS::setup() {
    setupCore();
    splitGuiMode_ = false;
    setupGUI();
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

    tsdt_pipeline.clearHistory();
    yolo_pipeline.clearHistory();
    tpdvs_gesture_pipeline.clearHistory();
}

//--------------------------------------------------------------
void ofxDVS::changePath(){
    thread.lock();
    thread.doChangePath = true;
    ofLog(OF_LOG_NOTICE, path);
    // update file path
    thread.path = path;

    // Detect format from extension
    {
        std::string ext;
        auto dotPos = path.rfind('.');
        if (dotPos != std::string::npos) ext = path.substr(dotPos);
        if (ext == ".aedat4")
            thread.fileFormat = AedatFormat::AEDAT4;
        else
            thread.fileFormat = AedatFormat::AEDAT31;
    }

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
    thread.aedat4Reader.reset();
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

    tsdt_pipeline.clearHistory();
    yolo_pipeline.clearHistory();
    tpdvs_gesture_pipeline.clearHistory();
    resetPlaybackTiming();
}

//--------------------------------------------------------------
void ofxDVS::openRecordingFileDb(){
    // File Recording Output — AEDAT4 via dv-processing (dataset subdirectory)
    string dir = getUserHomeDir();
    if(orginal){
        dir = dir+"/dataset/";
    }else{
        dir = dir+"/dataset/selected/";
    }
    time_t t = time(0);
    struct tm * now = localtime( & t );
    char buffer [80];
    strftime(buffer, 80, "ofxDVS_%Y-%m-%d-%I_%M_%S.aedat4", now);
    string filename = dir + "/" + buffer;

    std::string camName = chipIDToName(chipId, false);
    auto cfg = dv::io::MonoCameraWriter::EventOnlyConfig(camName,
                   cv::Size(sizeX, sizeY));
    aedat4Writer_ = std::make_unique<dv::io::MonoCameraWriter>(filename, cfg);
    ofLogNotice() << "[Recording] Opened AEDAT4 file: " << filename;
}

//--------------------------------------------------------------
void ofxDVS::openRecordingFile(){
    // File Recording Output — AEDAT4 via dv-processing
    string dir = getUserHomeDir();
    time_t t = time(0);
    struct tm * now = localtime( & t );
    char buffer [80];
    strftime(buffer, 80, "ofxDVS_%Y-%m-%d-%I_%M_%S.aedat4", now);
    string filename = dir + "/" + buffer;

    std::string camName = chipIDToName(chipId, false);
    auto cfg = dv::io::MonoCameraWriter::EventOnlyConfig(camName,
                   cv::Size(sizeX, sizeY));
    aedat4Writer_ = std::make_unique<dv::io::MonoCameraWriter>(filename, cfg);
    ofLogNotice() << "[Recording] Opened AEDAT4 file: " << filename;
}

//--------------------------------------------------------------
void ofxDVS::changeRecordingStatusDb(){
    apsStatus = false; // no APS
    dvsStatus = true; // yes DVS
    imuStatus = true; // yes IMU
    if(isRecording){
        isRecording = false;
        aedat4Writer_.reset();
        ofLog(OF_LOG_NOTICE, "Stop recording\n");
    }else{
        openRecordingFileDb();
        isRecording = true;
        ofLog(OF_LOG_NOTICE, "Start recording\n");
    }
}


//--------------------------------------------------------------
void ofxDVS::changeRecordingStatus(){
    if(isRecording){
        isRecording = false;
        aedat4Writer_.reset();
        ofLog(OF_LOG_NOTICE, "Stop recording\n");
    }else{
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

    return(true);

}


//--------------------------------------------------------------
void ofxDVS::update() {

    if(paused == false){
        // Copy data from usbThread
        // 1) take ownership quickly
        std::vector<caerEventPacketContainer> local;
        thread.lock();

        int newSizeX = -1, newSizeY = -1;
        if ((thread.deviceReady || thread.fileInputReady) &&
            (thread.sizeX != sizeX || thread.sizeY != sizeY)) {
            newSizeX = thread.sizeX;
            newSizeY = thread.sizeY;
        }

        // Only pull new packets from the USB thread when the backlog is empty.
        // This prevents the backlog from growing unboundedly: the USB thread
        // reads ahead at disk speed (filling MAX_QUEUE=64 packets = 3.2 s of
        // data) while the timing gate only releases ~1 packet per 50 ms.
        // Without this guard every frame adds 63 future packets to the backlog.
        if (backlog_.empty()) {
            local.swap(thread.container);   // O(1) swap, clears producer queue
        }
        thread.unlock();

        // Drain the backlog before touching USB packets.
        if (!backlog_.empty()) {
            local.assign(backlog_.begin(), backlog_.end());
            backlog_.clear();
            // Packets still too early will be re-deferred into backlog_ below.
        }

        if (newSizeX > 0) {
            int oldSizeX = sizeX;
            sizeX = newSizeX;
            sizeY = newSizeY;

            // Free and re-allocate raw 2D sensor arrays
            auto free2d_long = [](long**& arr, int W) {
                if (arr) { for (int i = 0; i < W; ++i) delete[] arr[i]; delete[] arr; arr = nullptr; }
            };
            auto free2d_float = [](float**& arr, int W) {
                if (arr) { for (int i = 0; i < W; ++i) delete[] arr[i]; delete[] arr; arr = nullptr; }
            };

            free2d_long(baFilterMap, oldSizeX);
            baFilterMap = new long*[sizeX];
            for (int i = 0; i < sizeX; ++i) baFilterMap[i] = new long[sizeY]();

            free2d_float(visualizerMap, oldSizeX);
            visualizerMap = new float*[sizeX];
            for (int i = 0; i < sizeX; ++i) visualizerMap[i] = new float[sizeY]();

            free2d_float(spikeFeatures, oldSizeX);
            spikeFeatures = new float*[sizeX];
            for (int i = 0; i < sizeX; ++i) spikeFeatures[i] = new float[sizeY]();

            free2d_float(surfaceMapLastTs, oldSizeX);
            surfaceMapLastTs = new float*[sizeX];
            for (int i = 0; i < sizeX; ++i) surfaceMapLastTs[i] = new float[sizeY]();

            free2d_float(spikeCountMap, oldSizeX);
            spikeCountMap = new float*[sizeX];
            for (int i = 0; i < sizeX; ++i) spikeCountMap[i] = new float[sizeY]();

            // Re-allocate flat vectors
            const size_t npix = (size_t)sizeX * sizeY;
            reconMap_.assign(npix, 0.0f);
            saeTimestamp_.assign(npix, 0);
            flowX_.assign(npix, 0.0f);
            flowY_.assign(npix, 0.0f);
            last_ts_map_.assign(npix, 0);
            hot_rate_count_.assign(npix, 0);
            hot_calib_count_.assign(npix, 0);
            hot_pixel_mask_.assign(npix, false);
            hot_calib_done_ = false;
            hot_calib_started_ = false;

            // Re-allocate ofImages and FBO
            imageGenerator.allocate(sizeX, sizeY, OF_IMAGE_COLOR);
            imagePolarity.allocate(sizeX, sizeY, OF_IMAGE_COLOR);
            reconImage_.allocate(sizeX, sizeY, OF_IMAGE_COLOR);
            flowImage_.allocate(sizeX, sizeY, OF_IMAGE_COLOR);
            meanRateImage.allocate(sizeX, sizeY, OF_IMAGE_COLOR_ALPHA);
            next_polarities_pixbuf.allocate(sizeX, sizeY, OF_IMAGE_COLOR);
            next_polarities.allocate(next_polarities_pixbuf);
            next_frame.allocate(next_polarities_pixbuf);
            fbo.allocate(sizeX, sizeY, GL_RGBA32F);
            tex = &fbo.getTexture();

            if (rectangularClusterTrackerEnabled) createRectangularClusterTracker();
            updateViewports();
        }

        // Check if the file looped (thread signalled a reset)
        if (thread.resetTimingFlag.exchange(false)) {
            timingInitialized_ = false;
            isStarted = false;

            // Reset all timestamp-dependent filter state so that
            // backward-jumping timestamps don't cause progressive
            // event starvation across file loops.
            std::fill(last_ts_map_.begin(), last_ts_map_.end(), 0);
            std::fill(hot_rate_count_.begin(), hot_rate_count_.end(), 0);
            hot_rate_window_start_ = 0;
            for (int i = 0; i < sizeX; ++i)
                for (int j = 0; j < sizeY; ++j)
                    baFilterMap[i][j] = 0;

            // Reset optical flow SAE timestamps
            std::fill(saeTimestamp_.begin(), saeTimestamp_.end(), 0);

            // Clear all NN pipeline histories for deterministic results
            tpdvs_gesture_pipeline.clearHistory();
            tsdt_pipeline.clearHistory();
            yolo_pipeline.clearHistory();
        }

        // Safety valve: backlog should never exceed MAX_QUEUE-1 with the
        // drain-first approach above, but guard against any edge case.
        if (backlog_.size() > 256) {
            ofLogWarning("ofxDVS") << "Backlog unexpectedly large ("
                << backlog_.size() << ") — clearing";
            for (auto &bp : backlog_) caerEventPacketContainerFree(bp);
            backlog_.clear();
            timingInitialized_ = false;
        }

        // 2) process WITHOUT holding the lock
        // Clear per-frame event buffers once (organizeData appends into them)
        packetsPolarity.clear();
        packetsImu6.clear();
        packetsFrames.clear();

        bool isFileMode = thread.fileInputReady;
        for (auto &packetContainer : local) {
            bool delpc = false;

            long packetTs = caerEventPacketContainerGetHighestEventTimestamp(packetContainer);

            // --- Timing gate for file playback ---
            if (isFileMode && packetTs != -1) {
                if (!timingInitialized_) {
                    fileTimeOrigin_ = packetTs;
                    wallTimeOrigin_ = ofGetElapsedTimeMicros();
                    timingInitialized_ = true;
                }
                int64_t fileOffset = packetTs - fileTimeOrigin_;

                // Detect timestamp backward jump (file looped mid-batch):
                // re-sync timing origin from this packet
                if (fileOffset < -1000000) {
                    fileTimeOrigin_ = packetTs;
                    wallTimeOrigin_ = ofGetElapsedTimeMicros();
                    fileOffset = 0;
                    isStarted = false;
                    // Discard stale deferred packets from previous loop
                    for (auto& bp : backlog_)
                        caerEventPacketContainerFree(bp);
                    backlog_.clear();
                    // Discard old-loop events accumulated earlier in this batch
                    packetsPolarity.clear();
                    packetsImu6.clear();
                    packetsFrames.clear();
                    // Reset timestamp-dependent filter state
                    std::fill(last_ts_map_.begin(), last_ts_map_.end(), 0);
                    std::fill(hot_rate_count_.begin(), hot_rate_count_.end(), 0);
                    hot_rate_window_start_ = 0;
                    for (int fi = 0; fi < sizeX; ++fi)
                        for (int fj = 0; fj < sizeY; ++fj)
                            baFilterMap[fi][fj] = 0;
                    std::fill(saeTimestamp_.begin(), saeTimestamp_.end(), 0);
                    // Clear NN pipeline histories
                    tpdvs_gesture_pipeline.clearHistory();
                    tsdt_pipeline.clearHistory();
                    yolo_pipeline.clearHistory();
                }

                if (playbackSpeed_ <= 0.0f) playbackSpeed_ = 0.01f;
                int64_t targetWallOffset = (int64_t)((double)fileOffset / (double)playbackSpeed_);
                int64_t wallNow = ofGetElapsedTimeMicros();
                if ((wallNow - wallTimeOrigin_) < targetWallOffset) {
                    // Too early — defer this and all subsequent packets
                    backlog_.push_back(packetContainer);
                    // defer remaining packets
                    for (size_t ri = (&packetContainer - &local[0]) + 1; ri < local.size(); ri++) {
                        backlog_.push_back(local[ri]);
                    }
                    break; // stop processing this frame
                }
            }

            size_t polarityOffset = packetsPolarity.size();
            delpc = organizeData(packetContainer);

            // Update time display
            if (packetTs != -1) {
                if (!isStarted || packetTs < started) { started = packetTs; isStarted = true; }
                unsigned long cur = packetTs - started;
                microseconds = cur - (minutes * 60) * 1e6 - seconds * 1e6;
                minutes      = (cur / 60e6);
                seconds      = (((int)cur % (int)60e6) / 1e6);
                hours        = 0;
                sprintf(timeString, " %02lu:%02lu:%02lu:%04lu", hours, minutes, seconds, microseconds);
            } else {
                sprintf(timeString, "%02u", 0u);
            }

            // AEDAT4 recording: write only events added by this packet
            if (isRecording && aedat4Writer_) {
                dv::EventStore store;
                for (size_t ei = polarityOffset; ei < packetsPolarity.size(); ++ei) {
                    const auto& e = packetsPolarity[ei];
                    if (!e.valid) continue;
                    store.emplace_back(e.timestamp, (int16_t)e.pos.x,
                                       (int16_t)e.pos.y, e.pol);
                }
                if (!store.isEmpty()) aedat4Writer_->writeEvents(store);
            }

            // Free the libcaer container now that we've copied all data out
            caerEventPacketContainerFree(packetContainer);
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

    // --- Event-driven image reconstruction (CPU per-pixel decay) ---
    if (drawRecon) {
        const int W = sizeX, H = sizeY;

        // 1) Decay all pixels towards zero every frame
        for (auto &v : reconMap_) v *= reconDecay;

        // 2) Apply events with spatial spread
        for (const auto &e : packetsPolarity) {
            if (!e.valid) continue;
            int cx = (int)e.pos.x, cy = (int)e.pos.y;
            float val = e.pol ? reconContrib : -reconContrib;
            for (int dy = -reconSpread; dy <= reconSpread; dy++) {
                for (int dx = -reconSpread; dx <= reconSpread; dx++) {
                    int px = cx + dx, py = cy + dy;
                    if (px < 0 || px >= W || py < 0 || py >= H) continue;
                    float dist = std::sqrt((float)(dx*dx + dy*dy));
                    if (dist > reconSpread) continue;
                    float w = 1.0f - dist / (reconSpread + 1.0f);
                    float &pix = reconMap_[py * W + px];
                    pix = std::clamp(pix + val * w, -1.0f, 1.0f);
                }
            }
        }

        // 3) Map to yellow (ON) / blue (OFF) colors
        unsigned char *raw = reconImage_.getPixels().getData();
        for (int i = 0; i < W * H; i++) {
            float v = reconMap_[i];
            int idx = i * 3;
            if (v > 0) {
                raw[idx]     = (unsigned char)(v * 255);
                raw[idx + 1] = (unsigned char)(v * 200);
                raw[idx + 2] = 0;
            } else {
                float a = -v;
                raw[idx]     = 0;
                raw[idx + 1] = (unsigned char)(a * 100);
                raw[idx + 2] = (unsigned char)(a * 255);
            }
        }
        reconImage_.update();
    }

    // --- Event-based optical flow (SAE + local plane fitting) ---
    {
        const int W = sizeX, H = sizeY;
        const int R = optFlowRadius;
        const int64_t dtThresh = (int64_t)optFlowDt_us;

        // Always update SAE with valid events (keeps the map warm)
        for (const auto &e : packetsPolarity) {
            if (!e.valid) continue;
            int ex = (int)e.pos.x, ey = (int)e.pos.y;
            if (ex >= 0 && ex < W && ey >= 0 && ey < H)
                saeTimestamp_[ey * W + ex] = e.timestamp;
        }

        if (drawOptFlow) {
            // Decay existing flow vectors
            for (int i = 0, n = W * H; i < n; i++) {
                flowX_[i] *= optFlowDecay;
                flowY_[i] *= optFlowDecay;
            }

            // Compute flow per valid event via local plane fitting
            for (const auto &e : packetsPolarity) {
                if (!e.valid) continue;
                int ex = (int)e.pos.x, ey = (int)e.pos.y;
                if (ex < R || ex >= W - R || ey < R || ey >= H - R) continue;

                int64_t t0 = e.timestamp;

                // Accumulate normal equations for t = a*dx + b*dy + c
                // Using Cramer's rule on the 3x3 system:
                //   [sum(dx^2)  sum(dx*dy) sum(dx) ] [a]   [sum(dx*t)]
                //   [sum(dx*dy) sum(dy^2)  sum(dy) ] [b] = [sum(dy*t)]
                //   [sum(dx)    sum(dy)    N       ] [c]   [sum(t)   ]
                double Sxx = 0, Syy = 0, Sxy = 0, Sx = 0, Sy = 0;
                double Sxt = 0, Syt = 0, St = 0;
                int N = 0;

                for (int dy = -R; dy <= R; dy++) {
                    for (int dx = -R; dx <= R; dx++) {
                        int64_t ts = saeTimestamp_[(ey + dy) * W + (ex + dx)];
                        if (ts == 0) continue;
                        int64_t age = t0 - ts;
                        if (age < 0 || age > dtThresh) continue;
                        double dt = (double)(ts - t0); // relative timestamp (negative or zero)
                        Sxx += dx * dx;
                        Syy += dy * dy;
                        Sxy += dx * dy;
                        Sx  += dx;
                        Sy  += dy;
                        Sxt += dx * dt;
                        Syt += dy * dt;
                        St  += dt;
                        N++;
                    }
                }

                if (N < 6) continue; // need enough neighbors for robust fit

                // Solve 3x3 system via Cramer's rule
                // M = [[Sxx, Sxy, Sx], [Sxy, Syy, Sy], [Sx, Sy, N]]
                double det = Sxx * (Syy * N - Sy * Sy)
                           - Sxy * (Sxy * N - Sy * Sx)
                           + Sx  * (Sxy * Sy - Syy * Sx);

                if (std::abs(det) < 1e-6) continue; // near-singular

                double detA = Sxt * (Syy * N - Sy * Sy)
                            - Sxy * (Syt * N - Sy * St)
                            + Sx  * (Syt * Sy - Syy * St);

                double detB = Sxx * (Syt * N - Sy * St)
                            - Sxt * (Sxy * N - Sy * Sx)
                            + Sx  * (Sxy * St - Syt * Sx);

                double a = detA / det; // dt/dx in microseconds/pixel
                double b = detB / det; // dt/dy in microseconds/pixel

                // Flow velocity: vx = -1/a, vy = -1/b (pixels/us) → convert to px/s
                if (std::abs(a) < 1e-10 && std::abs(b) < 1e-10) continue;

                float vx = 0, vy = 0;
                // Use the gradient vector directly: v = -(a,b) / (a^2+b^2) * 1e6
                double denom = a * a + b * b;
                if (denom < 1e-20) continue;
                vx = (float)(-a / denom * 1e6);
                vy = (float)(-b / denom * 1e6);

                float mag = std::sqrt(vx * vx + vy * vy);
                if (mag > optFlowMaxSpeed * 5.0f) continue; // reject outliers

                int idx = ey * W + ex;
                flowX_[idx] = vx;
                flowY_[idx] = vy;
            }

            // Render flow as HSV color wheel
            unsigned char *raw = flowImage_.getPixels().getData();
            for (int i = 0; i < W * H; i++) {
                float vx = flowX_[i], vy = flowY_[i];
                float mag = std::sqrt(vx * vx + vy * vy);
                int idx3 = i * 3;
                if (mag < 0.5f) {
                    raw[idx3] = raw[idx3 + 1] = raw[idx3 + 2] = 0;
                    continue;
                }

                // Hue from angle [0, 360)
                float hue = std::atan2(vy, vx) * (180.0f / M_PI) + 180.0f;
                float sat = 1.0f;
                float val = std::min(mag / optFlowMaxSpeed, 1.0f);

                // HSV to RGB
                float c = val * sat;
                float x = c * (1.0f - std::abs(std::fmod(hue / 60.0f, 2.0f) - 1.0f));
                float m = val - c;
                float r, g, b;
                if      (hue < 60)  { r = c; g = x; b = 0; }
                else if (hue < 120) { r = x; g = c; b = 0; }
                else if (hue < 180) { r = 0; g = c; b = x; }
                else if (hue < 240) { r = 0; g = x; b = c; }
                else if (hue < 300) { r = x; g = 0; b = c; }
                else                { r = c; g = 0; b = x; }
                raw[idx3]     = (unsigned char)((r + m) * 255);
                raw[idx3 + 1] = (unsigned char)((g + m) * 255);
                raw[idx3 + 2] = (unsigned char)((b + m) * 255);
            }
            flowImage_.update();
        }
    }

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

        // Debug: log tracker state every ~60 frames
        static int dbg_ctr = 0;
        if (++dbg_ctr % 60 == 0) {
            ofLogNotice() << "[Tracker] valid_events=" << inq.size()
                          << " clusters=" << rectangularClusterTracker->getNumClusters()
                          << " latest_ts=" << latest_ts;
        }
    }

    // ---- TSDT: push events and run inference (synchronous) ----
    if (!packetsPolarity.empty()) {
        tsdt_pipeline.pushEvents(packetsPolarity, sizeX, sizeY);
        tpdvs_gesture_pipeline.pushEvents(packetsPolarity, sizeX, sizeY);

        if (tsdtEnabled && tsdt_pipeline.isLoaded()) {
            tsdt_pipeline.infer(sizeX, sizeY);
        }
        if (tpdvsGestureEnabled && tpdvs_gesture_pipeline.isLoaded()) {
            tpdvs_gesture_pipeline.infer(sizeX, sizeY);
        }
    }

    //GUI
    if (!splitGuiMode_) updateGUI();

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

    size_t nClusters = rectangularClusterTracker->getNumClusters();

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
// updateGUI() — update f1 panel and text widgets (call from control window)
void ofxDVS::updateGUI() {
    f1->update();
    myTextTimer->setText(timeString);
    myTempReader->setText(to_string((int)(imuTemp)));
}

//--------------------------------------------------------------
// drawViewer() — visualization only (spikes, images, overlays, labels)
void ofxDVS::drawViewer() {

    myCam.begin();
    ofTranslate(ofPoint(-ofGetWidth()/2,-ofGetHeight()/2));
    drawFrames();
    drawImageGenerator();
    if (drawRecon) {
        reconImage_.draw(0, 0, ofGetWidth(), ofGetHeight());
    }
    if (drawOptFlow) {
        flowImage_.draw(0, 0, ofGetWidth(), ofGetHeight());
    }
    drawSpikes();         // if dvs.doDrawSpikes
    drawImu6();

    drawRectangularClusterTracker();

    // YOLO detections from async worker
    if (nnEnabled && yolo_worker.hasResult()) {
        yolo_pipeline.detections() = yolo_worker.lastResult();
        if (yolo_pipeline.probesEnabled()) {
            updateProbeTextures_();
            updateSpikeTextures_();
        }
    }
    yolo_pipeline.drawDetections(sizeX, sizeY);

    // SNN YOLO detections (cyan boxes)
    if (snnYoloEnabled)
        snn_yolo_pipeline.drawDetections(sizeX, sizeY);

    // TPDVSGesture label + probe textures
    if (tpdvsGestureEnabled) {
        if (tpdvs_gesture_pipeline.probesEnabled())
            updateGestureTextures_();
        tpdvs_gesture_pipeline.drawLabel();
    }

    // TSDT label
    if (tsdtEnabled) {
        tsdt_pipeline.drawLabel();
    }

    myCam.end();

    // NN visualizations (VTEI, probes, SNN spikes) are shown in the dedicated NNViz window.

    // Debug: draw cluster count outside camera transform (reliable 2D text)
    if (rectangularClusterTrackerEnabled && rectangularClusterTracker) {
        size_t n = rectangularClusterTracker->getNumClusters();
        char buf[64];
        snprintf(buf, sizeof(buf), "Clusters: %zu", n);
        ofDrawBitmapStringHighlight(buf, 20, ofGetHeight() - 30,
                                    ofColor(0,0,0,180), ofColor(255,215,0));
    }

    drawMouseDistanceToSpikes();

    // Deferred recorder start: grab one frame first to get the true
    // pixel dimensions (may differ from ofGetWidth due to HiDPI scaling),
    // then configure ffmpeg to match exactly.
    if (videoRecPending_) {
        videoRecPending_ = false;
        int reqW = ofGetWidth()  & ~1;
        int reqH = ofGetHeight() & ~1;
        rec_grab_.grabScreen(0, 0, reqW, reqH);
        rec_w_ = (int)rec_grab_.getWidth()  & ~1;
        rec_h_ = (int)rec_grab_.getHeight() & ~1;
        int nCh = rec_grab_.getPixels().getNumChannels();
        ofImageType imgType = (nCh == 4) ? OF_IMAGE_COLOR_ALPHA : OF_IMAGE_COLOR;
        videoRecording_ = true;

        videoRecorder_.setup(true, false,
                             glm::vec2(rec_w_, rec_h_),
                             videoRecFps_, 8000);
        videoRecorder_.setOutputPath(videoRecPath_);
        videoRecorder_.setVideoCodec("libx264");
        videoRecorder_.setInputPixelFormat(imgType);
        videoRecorder_.setOverWrite(true);
        videoRecorder_.startCustomRecord();
        ofLogNotice() << "[Video] Recording to " << videoRecPath_
                      << " (" << rec_w_ << "x" << rec_h_
                      << ", " << nCh << "ch)";
        // First frame already grabbed — send it
        videoRecorder_.addFrame(rec_grab_.getPixels());
    }

    if (videoRecording_ && !videoRecPaused_ && !videoRecPending_) {
        rec_grab_.grabScreen(0, 0, rec_w_, rec_h_);
        videoRecorder_.addFrame(rec_grab_.getPixels());
    }
}

//--------------------------------------------------------------
// drawControls() — f1 panel (other panels auto-draw via ofEvents)
void ofxDVS::drawControls() {
    if (drawGui) {
        f1->draw();
    }
}

//--------------------------------------------------------------
// draw() — backward-compatible single-window draw
void ofxDVS::draw() {
    drawViewer();
    if (!splitGuiMode_) drawControls();
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
    if (!baFilterEnabled_) return;

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
        ofFileDialogResult result = ofSystemLoadDialog("Load recording (.aedat / .aedat4)");
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
    ofFileDialogResult result = ofSystemLoadDialog("Load recording (.aedat / .aedat4)");
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

    if (videoRecorder_.isRecording()) {
        videoRecorder_.stop();
        videoRecording_ = false;
    }

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
        aedat4Writer_.reset();
    }
    thread.aedat4Reader.reset();

    ofLogNotice() << "[ofxDVS] exit: done";
}

//--------------------------------------------------------------
void ofxDVS::startVideoRecording_() {
    videoRecPending_ = true;
}

//--------------------------------------------------------------
void ofxDVS::stopVideoRecording_() {
    videoRecording_ = false;
    videoRecPaused_ = false;
    rec_w_ = 0;
    rec_h_ = 0;
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
        imageGenerator.draw(0, 0, ofGetWidth(), ofGetHeight());
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

        // --- VTEI-based inference via async workers ---
        bool needVTEI = (nnEnabled && yolo_pipeline.isLoaded());
        if (needVTEI && newImageGen) {
            // Build VTEI tensor on main thread (fast, uses pipeline pre-allocated buffers)
            // buildVTEI returns a const ref to internal buffer; copy once for the worker
            auto vtei = std::vector<float>(yolo_pipeline.buildVTEI(
                packetsPolarity, surfaceMapLastTs,
                imageGenerator.getPixels(), sizeX, sizeY));

            int sw = sizeX, sh = sizeY;

            // Submit YOLO inference (non-blocking; dropped if worker is busy)
            // Move vtei into the lambda to avoid a second copy
            if (nnEnabled && yolo_pipeline.isLoaded()) {
                yolo_worker.submit([this, vtei = std::move(vtei), sw, sh]() -> std::vector<dvs::YoloDet> {
                    yolo_pipeline.infer(vtei, sw, sh);
                    return yolo_pipeline.detections();
                });
            }

            // Always upload VTEI textures — NNViz window reads them regardless of show flag
            updateVteiTextures_(yolo_pipeline.lastVteiSensor(),
                                yolo_pipeline.lastVteiW(),
                                yolo_pipeline.lastVteiH());
        }

        // --- SNN YOLO (twl_spike_yolo / yolo8n_gen1): one 8-ms bin per update ---
        if (snnYoloEnabled && snn_yolo_pipeline.isLoaded()) {
            snn_yolo_pipeline.pushEvents(packetsPolarity, sizeX, sizeY);
            snn_yolo_pipeline.maybeInfer(sizeX, sizeY);
        }
    }
}

// ---- updateProbeTextures_ (hot colormap: black→red→yellow→white) ----
static void hotColor(float v, float& r, float& g, float& b) {
    r = std::clamp(v * 3.f,        0.f, 1.f);
    g = std::clamp(v * 3.f - 1.f,  0.f, 1.f);
    b = std::clamp(v * 3.f - 2.f,  0.f, 1.f);
}

void ofxDVS::updateProbeTextures_() {
    const auto& probes = yolo_pipeline.probeResults();
    ofFloatPixels pix;
    for (int p = 0; p < dvs::YoloPipeline::NUM_PROBES; ++p) {
        const auto& pm = probes[p];
        if (pm.W == 0 || pm.H == 0 || pm.avg.empty()) continue;
        pix.allocate(pm.W, pm.H, OF_IMAGE_COLOR);
        float* dst = pix.getData();
        for (int i = 0; i < pm.W * pm.H; ++i) {
            hotColor(pm.avg[i], dst[i*3+0], dst[i*3+1], dst[i*3+2]);
        }
        probeTextures_[p].loadData(pix);
    }
}

void ofxDVS::updateGestureTextures_() {
    constexpr int NP = dvs::TsdtPipeline::NUM_PROBES;
    constexpr int NN = dvs::TsdtPipeline::NUM_RASTER_NEURONS;
    constexpr int NH = dvs::TsdtPipeline::RASTER_HISTORY;

    // ---- Probe activation maps (hot colormap) ----
    const auto& probes = tpdvs_gesture_pipeline.probeResults();
    ofFloatPixels pix;
    for (int p = 0; p < NP; ++p) {
        const auto& pm = probes[p];
        if (pm.W == 0 || pm.H == 0 || pm.avg.empty()) continue;
        pix.allocate(pm.W, pm.H, OF_IMAGE_COLOR);
        float* dst = pix.getData();
        for (int i = 0; i < pm.W * pm.H; ++i)
            hotColor(pm.avg[i], dst[i*3+0], dst[i*3+1], dst[i*3+2]);
        gestureProbeTextures_[p].loadData(pix);
    }

    // ---- Spike rasterplot textures (green-on-black, NH×NN pixels) ----
    // Each pixel column = one time step, each row = one neuron.
    const auto& raster = tpdvs_gesture_pipeline.rasterData();
    int pos = tpdvs_gesture_pipeline.rasterPos();  // next write position = oldest slot
    ofFloatPixels rpix;
    rpix.allocate(NH, NN, OF_IMAGE_COLOR);
    float* rd = rpix.getData();
    for (int p = 0; p < NP; ++p) {
        for (int t = 0; t < NH; ++t) {
            int slot = (pos + t) % NH;  // oldest first → leftmost
            const auto& row = raster[p][slot];
            for (int n = 0; n < NN; ++n) {
                int pi = (n * NH + t) * 3;
                float s = row[n];
                rd[pi+0] = 0.f;
                rd[pi+1] = s;          // green channel for spikes
                rd[pi+2] = s * 0.3f;  // slight teal tint
            }
        }
        gestureRasterTextures_[p].loadData(rpix);
    }
}

// ---- drawProbeActivations_ ----
void ofxDVS::drawProbeActivations_() {
    const auto& probes = yolo_pipeline.probeResults();

    const int tileH  = 88;
    const int gap    = 5;
    const int labelH = 14;
    const int padX   = 8, padY = 6;

    // Compute strip width based on first valid probe aspect ratio
    int tileW = tileH;  // default square
    for (int p = 0; p < dvs::YoloPipeline::NUM_PROBES; ++p) {
        if (probes[p].W > 0 && probes[p].H > 0) {
            tileW = (int)((float)probes[p].W / probes[p].H * tileH);
            break;
        }
    }

    const int N      = dvs::YoloPipeline::NUM_PROBES;
    const int totalW = N * tileW + (N-1) * gap + 2 * padX;
    const int totalH = tileH + labelH + 2 * padY;

    int vw = ofGetWidth(), vh = ofGetHeight();
    int ox = (vw - totalW) / 2;
    // Place above VTEI strip if both visible, otherwise at bottom
    int oy = showVteiChannels_
        ? vh - (tileH + labelH + 2*padY + 4) - totalH - 8
        : vh - totalH - 4;

    ofPushStyle();
    ofDisableDepthTest();

    ofSetColor(10, 10, 10, 180);
    ofDrawRectRounded(ox, oy, totalW, totalH, 4);

    for (int p = 0; p < N; ++p) {
        const auto& pm = probes[p];
        float tx = ox + padX + p * (tileW + gap);
        float ty = oy + padY;
        if (pm.W > 0 && pm.H > 0 && probeTextures_[p].isAllocated()) {
            ofSetColor(255);
            probeTextures_[p].draw(tx, ty, tileW, tileH);
        } else {
            ofSetColor(40, 40, 40);
            ofDrawRectangle(tx, ty, tileW, tileH);
        }
        ofSetColor(220, 220, 220);
        ofDrawBitmapString(pm.label, tx + 2, ty + tileH + labelH - 2);
    }

    ofPopStyle();
}

// ---- drawNNViz ----
// Draws VTEI + 3 groups of 4 analog probe rows + 3 groups of 4 SNN spike rows.
//
// Groups (4 probes each):
//   0: LSTM hidden state h_t (P2/P3/P4/P5 gate Concat)
//   1: LSTM cell state c_t = f*c_prev + i*g (c2/c3/c4/c5)
//   2: Feedforward conv features (Stem/SPPF/NkP3/NkP5)
void ofxDVS::drawNNViz() {
    static bool logged = false;
    if (!logged) {
        logged = true;
        ofLogNotice() << "[NNViz] probesEnabled=" << yolo_pipeline.probesEnabled()
                      << " showProbeActivations=" << showProbeActivations_
                      << " showSnnSpikes=" << showSnnSpikes_
                      << " showVteiChannels=" << showVteiChannels_
                      << " nnEnabled=" << nnEnabled;
    }

    ofBackground(18, 18, 22);
    ofDisableDepthTest();

    static const char* GROUP_HEADERS[3] = {
        "LSTM hidden (h_t)",
        "LSTM cell state (c_t = f*c_prev + i*g)",
        "Conv features",
    };
    static const char* vlabels[5]  = {"Pos","Neg","TimeSurf","Early","Late"};
    static const ofColor vtints[5] = {
        {60,220,80},{220,60,60},{60,180,255},{255,200,0},{255,120,0}
    };

    constexpr int PG        = 4;     // probes per group
    constexpr int NGROUPS   = dvs::YoloPipeline::NUM_PROBES / PG;  // 3
    const int tileH_probe   = 80;
    const int tileH_spike   = 64;
    const int tileH_vtei    = 80;
    const int labelH        = 14;
    const int padX = 8, padY = 6;
    const int gap           = 5;
    const int headerH       = 14;
    const int groupGap      = 8;
    const int stripGap      = 14;

    int vw = ofGetWidth();
    int cy = 10;

    const auto& probes = yolo_pipeline.probeResults();

    // Probe tile dimensions (aspect from first valid probe)
    int probeTileW = tileH_probe;
    for (int p = 0; p < dvs::YoloPipeline::NUM_PROBES; ++p)
        if (probes[p].W > 0 && probes[p].H > 0)
            { probeTileW = (int)((float)probes[p].W / probes[p].H * tileH_probe); break; }
    int spikeTileW = (int)((float)probeTileW / tileH_probe * tileH_spike);
    int probeRowH  = tileH_probe + labelH + 2*padY;
    int spikeRowH  = tileH_spike + labelH + 2*padY;

    // Lambda: draw one row of PG tiles from a probe group
    auto drawRow = [&](int groupIdx, int tileH, int tileW, int rowH, auto& texArr, bool isSpike) {
        int rowW = PG*(tileW+gap) - gap + 2*padX;
        int ox   = std::max(2, (vw - rowW) / 2);
        ofSetColor(isSpike ? ofColor(8,12,28,200) : ofColor(10,10,10,200));
        ofDrawRectRounded(ox, cy, rowW, rowH, 4);
        for (int p = 0; p < PG; ++p) {
            int pi = groupIdx*PG + p;
            const auto& pm = probes[pi];
            int tx = ox + padX + p*(tileW+gap);
            int ty = cy + padY;
            if (pm.W > 0 && pm.H > 0 && texArr[pi].isAllocated()) {
                ofSetColor(255);
                texArr[pi].draw(tx, ty, tileW, tileH);
            } else {
                ofSetColor(isSpike ? ofColor(20,22,50) : ofColor(40,40,40));
                ofDrawRectangle(tx, ty, tileW, tileH);
            }
            if (isSpike) {
                char buf[32];
                std::snprintf(buf, sizeof(buf), "%s %.1f%%", pm.label.c_str(), spikeRates_[pi]*100.f);
                ofSetColor(0, 220, 100);
                ofDrawBitmapString(buf, tx+2, ty+tileH+labelH-2);
            } else {
                ofSetColor(220, 220, 220);
                ofDrawBitmapString(pm.label, tx+2, ty+tileH+labelH-2);
            }
        }
        cy += rowH;
    };

    // ---- VTEI input channels ----
    if (showVteiChannels_) {
        int W = yolo_pipeline.lastVteiW(), H = yolo_pipeline.lastVteiH();
        if (W > 0 && H > 0) {
            int tileW  = (int)((float)W / H * tileH_vtei);
            int totalW = 5*tileW + 4*gap + 2*padX;
            int totalH = tileH_vtei + labelH + 2*padY;
            int ox = std::max(2, (vw - totalW) / 2);
            ofSetColor(10, 10, 10, 200);
            ofDrawRectRounded(ox, cy, totalW, totalH, 4);
            for (int c = 0; c < 5; ++c) {
                int tx = ox + padX + c*(tileW+gap), ty = cy + padY;
                ofSetColor(vtints[c]);
                if (vteiTextures_[c].isAllocated())
                    vteiTextures_[c].draw(tx, ty, tileW, tileH_vtei);
                ofDrawBitmapString(vlabels[c], tx+2, ty+tileH_vtei+labelH-2);
            }
            cy += totalH + stripGap;
        }
    }

    // ---- Analog probe activations (hot colormap), 3 groups of 4 ----
    if (showProbeActivations_ && yolo_pipeline.probesEnabled()) {
        for (int g = 0; g < NGROUPS; ++g) {
            int rowW = PG*(probeTileW+gap) - gap + 2*padX;
            ofSetColor(160, 160, 160);
            ofDrawBitmapString(GROUP_HEADERS[g], std::max(2, (vw-rowW)/2), cy+headerH-2);
            cy += headerH;
            drawRow(g, tileH_probe, probeTileW, probeRowH, probeTextures_, false);
            cy += (g < NGROUPS-1) ? groupGap : stripGap;
        }
    }

    // ---- SNN spike maps (binarized, green-on-black), 3 groups of 4 ----
    if (showSnnSpikes_ && yolo_pipeline.probesEnabled()) {
        for (int g = 0; g < NGROUPS; ++g) {
            int rowW = PG*(spikeTileW+gap) - gap + 2*padX;
            char hdr[64];
            std::snprintf(hdr, sizeof(hdr), "SNN: %s", GROUP_HEADERS[g]);
            ofSetColor(0, 160, 80);
            ofDrawBitmapString(hdr, std::max(2, (vw-rowW)/2), cy+headerH-2);
            cy += headerH;
            drawRow(g, tileH_spike, spikeTileW, spikeRowH, spikeTextures_, true);
            cy += (g < NGROUPS-1) ? groupGap : stripGap;
        }
    }

    // ---- TPDVSGesture: 8 membrane probe tiles + 8 spike rasterplots ----
    if (tpdvs_gesture_pipeline.probesEnabled()) {
        constexpr int GP   = dvs::TsdtPipeline::NUM_PROBES;        // 8
        constexpr int NH   = dvs::TsdtPipeline::RASTER_HISTORY;    // 100
        constexpr int NN   = dvs::TsdtPipeline::NUM_RASTER_NEURONS; // 10
        constexpr int GCOLS = 4;  // tiles per row

        const auto& gprobes = tpdvs_gesture_pipeline.probeResults();

        const int gtileH  = 56;
        const int gtileW  = gtileH;   // all layers are square feature maps
        const int rasterW = 120;      // display width for rasterplot (NH columns scaled)
        const int rasterH = NN * 6;   // 6px per neuron row

        // Row width is set by 4 probe tiles
        int growW = GCOLS*(gtileW+gap) - gap + 2*padX;
        int ox    = std::max(2, (vw - growW) / 2);

        // Section header
        cy += groupGap;
        ofSetColor(255, 180, 60);
        ofDrawBitmapString("Gesture SNN — membrane potential (L0-L7, all 8 layers)",
                           ox, cy + headerH - 2);
        cy += headerH;

        // 2 rows of 4 probe tiles
        for (int row = 0; row < 2; ++row) {
            int rowH = gtileH + labelH + 2*padY;
            ofSetColor(10, 10, 10, 200);
            ofDrawRectRounded(ox, cy, growW, rowH, 4);
            for (int col = 0; col < GCOLS; ++col) {
                int p  = row * GCOLS + col;
                const auto& pm = gprobes[p];
                int tx = ox + padX + col*(gtileW+gap);
                int ty = cy + padY;
                if (pm.W > 0 && pm.H > 0 && gestureProbeTextures_[p].isAllocated()) {
                    ofSetColor(255);
                    gestureProbeTextures_[p].draw(tx, ty, gtileW, gtileH);
                } else {
                    ofSetColor(40, 40, 40);
                    ofDrawRectangle(tx, ty, gtileW, gtileH);
                }
                ofSetColor(220, 220, 220);
                ofDrawBitmapString(pm.label, tx+2, ty+gtileH+labelH-2);
            }
            cy += rowH + (row == 0 ? groupGap : 0);
        }

        // Section header for rasterplot
        cy += groupGap;
        ofSetColor(80, 220, 120);
        ofDrawBitmapString("Gesture SNN — spike rasterplot (10 neurons/layer, last 100 steps)",
                           ox, cy + headerH - 2);
        cy += headerH;

        // 2 rows of 4 rasterplot panels
        int rrW = GCOLS * (rasterW+gap) - gap + 2*padX;
        int rrox = std::max(2, (vw - rrW) / 2);

        for (int row = 0; row < 2; ++row) {
            int rowH = rasterH + labelH + 2*padY;
            ofSetColor(8, 12, 20, 220);
            ofDrawRectRounded(rrox, cy, rrW, rowH, 4);
            for (int col = 0; col < GCOLS; ++col) {
                int p  = row * GCOLS + col;
                int tx = rrox + padX + col*(rasterW+gap);
                int ty = cy + padY;
                if (gestureRasterTextures_[p].isAllocated()) {
                    ofSetColor(255);
                    gestureRasterTextures_[p].draw(tx, ty, rasterW, rasterH);
                } else {
                    ofSetColor(20, 30, 20);
                    ofDrawRectangle(tx, ty, rasterW, rasterH);
                }
                // Neuron-index axis: draw faint horizontal lines between neurons
                ofSetColor(40, 60, 40, 120);
                for (int n = 1; n < NN; ++n) {
                    int ry = ty + n * rasterH / NN;
                    ofDrawLine(tx, ry, tx + rasterW, ry);
                }
                // Label
                ofSetColor(80, 220, 120);
                const auto& pm = gprobes[p];
                ofDrawBitmapString(pm.label, tx+2, ty+rasterH+labelH-2);
            }
            cy += rowH + (row == 0 ? groupGap : 0);
        }
        (void)NH; (void)NN;
    }

    (void)cy;
}

// ---- updateVteiTextures_ ----
void ofxDVS::updateVteiTextures_(const std::vector<float>& chw5, int W, int H) {
    if (W == 0 || H == 0 || chw5.size() < (size_t)5 * W * H) return;
    const size_t plane = (size_t)W * H;
    ofFloatPixels pix;
    pix.allocate(W, H, OF_IMAGE_GRAYSCALE);
    float* dst = pix.getData();
    for (int c = 0; c < 5; ++c) {
        const float* src = chw5.data() + c * plane;
        if (c < 3) {
            // Channels 0-2 are already 0..1
            std::copy(src, src + plane, dst);
        } else {
            // Channels 3-4 are ternary {-1, 0, +1} — shift to 0..1 for display
            for (size_t i = 0; i < plane; ++i)
                dst[i] = src[i] * 0.5f + 0.5f;
        }
        vteiTextures_[c].loadData(pix);
    }
}

// ---- drawVteiChannels_ ----
void ofxDVS::drawVteiChannels_() {
    int W = yolo_pipeline.lastVteiW();
    int H = yolo_pipeline.lastVteiH();
    if (W == 0 || H == 0) return;

    static const char* labels[5] = {"Pos", "Neg", "TimeSurf", "Early", "Late"};
    static const ofColor tints[5] = {
        ofColor(60,  220, 80),   // pos  — green
        ofColor(220, 60,  60),   // neg  — red
        ofColor(60,  180, 255),  // time — cyan-blue
        ofColor(255, 200, 0),    // early bin — yellow
        ofColor(255, 120, 0),    // late bin  — orange
    };

    const int tileH  = 88;
    const int tileW  = (int)((float)W / H * tileH);
    const int gap    = 5;
    const int labelH = 14;
    const int padX   = 8, padY = 6;
    const int totalW = 5 * tileW + 4 * gap + 2 * padX;
    const int totalH = tileH + labelH + 2 * padY;

    int vw = ofGetWidth(), vh = ofGetHeight();
    int ox = (vw - totalW) / 2;
    int oy = vh - totalH - 4;

    ofPushStyle();
    ofDisableDepthTest();

    // Semi-transparent background
    ofSetColor(10, 10, 10, 180);
    ofDrawRectRounded(ox, oy, totalW, totalH, 4);

    for (int c = 0; c < 5; ++c) {
        float tx = ox + padX + c * (tileW + gap);
        float ty = oy + padY;
        ofSetColor(tints[c]);
        vteiTextures_[c].draw(tx, ty, tileW, tileH);
        ofSetColor(tints[c]);
        ofDrawBitmapString(labels[c], tx + 2, ty + tileH + labelH - 2);
    }

    ofPopStyle();
}


// ---- updateSpikeTextures_ ----
// Threshold probe activations (already normalized [0,1]) → binary green/black textures.
// Also computes spike rate (fraction of pixels above threshold) per probe.
void ofxDVS::updateSpikeTextures_() {
    const auto& probes = yolo_pipeline.probeResults();
    ofFloatPixels pix;
    for (int p = 0; p < dvs::YoloPipeline::NUM_PROBES; ++p) {
        const auto& pm = probes[p];
        if (pm.W == 0 || pm.H == 0 || pm.avg.empty()) {
            spikeRates_[p] = 0.f;
            continue;
        }
        const size_t n = pm.avg.size();
        pix.allocate(pm.W, pm.H, OF_IMAGE_COLOR);
        float* dst = pix.getData();
        int fired = 0;
        for (size_t i = 0; i < n; ++i) {
            if (pm.avg[i] >= snnSpikeThreshold_) {
                dst[i*3+0] = 0.0f;  dst[i*3+1] = 1.0f;  dst[i*3+2] = 0.5f;  // green
                ++fired;
            } else {
                dst[i*3+0] = 0.06f; dst[i*3+1] = 0.06f; dst[i*3+2] = 0.06f; // near-black
            }
        }
        spikeRates_[p] = (float)fired / (float)n;
        spikeTextures_[p].loadData(pix);
    }
}

// ---- drawSnnSpikes_ ----
// Draws binary spike maps stacked above the analog probe strip (and above VTEI if visible).
// Green pixels = neuron fired (activation ≥ threshold); dark = silent.
void ofxDVS::drawSnnSpikes_() {
    const auto& probes = yolo_pipeline.probeResults();

    const int tileH  = 72;
    const int gap    = 5;
    const int labelH = 14;
    const int padX   = 8, padY = 6;

    int tileW = tileH;
    for (int p = 0; p < dvs::YoloPipeline::NUM_PROBES; ++p) {
        if (probes[p].W > 0 && probes[p].H > 0) {
            tileW = (int)((float)probes[p].W / probes[p].H * tileH);
            break;
        }
    }

    const int N      = dvs::YoloPipeline::NUM_PROBES;
    const int totalW = N * tileW + (N-1) * gap + 2 * padX;
    const int totalH = tileH + labelH + 2 * padY;

    int vw = ofGetWidth(), vh = ofGetHeight();
    int ox = (vw - totalW) / 2;

    // Stack above any already-visible strips (VTEI=88px strip, probe=88px strip)
    int from_bottom = 4;
    if (showVteiChannels_)     from_bottom += (88 + 14 + 12) + 8;
    if (showProbeActivations_) from_bottom += (88 + 14 + 12) + 8;
    int oy = vh - from_bottom - totalH;

    ofPushStyle();
    ofDisableDepthTest();

    // Dark blue-tinted background — visually distinct from the hot-colormap probe strip
    ofSetColor(8, 12, 28, 200);
    ofDrawRectRounded(ox, oy, totalW, totalH, 4);

    for (int p = 0; p < N; ++p) {
        const auto& pm = probes[p];
        float tx = ox + padX + p * (tileW + gap);
        float ty = oy + padY;

        if (pm.W > 0 && pm.H > 0 && spikeTextures_[p].isAllocated()) {
            ofSetColor(255);
            spikeTextures_[p].draw(tx, ty, tileW, tileH);
        } else {
            ofSetColor(20, 22, 50);
            ofDrawRectangle(tx, ty, tileW, tileH);
        }

        char buf[32];
        std::snprintf(buf, sizeof(buf), "P%d %.1f%%", p + 2, spikeRates_[p] * 100.f);
        ofSetColor(0, 220, 100);
        ofDrawBitmapString(buf, tx + 2, ty + tileH + labelH - 2);
    }

    ofPopStyle();
}

//--------------------------------------------------------------
float ofxDVS::getImuTemp(){
    return(imuTemp);
}

//--------------------------------------------------------------
void ofxDVS::setPlaybackSpeed(float sliderPos){
    float oldSpeed = playbackSpeed_;
    speedSliderPos_ = sliderPos;
    playbackSpeed_ = std::pow(10.0f, sliderPos);
    if (playbackSpeed_ < 0.01f) playbackSpeed_ = 0.01f;

    // Continuity: recalculate wallTimeOrigin_ so current file position stays put
    if (timingInitialized_ && oldSpeed > 0.0f) {
        int64_t now = ofGetElapsedTimeMicros();
        double currentFileOffset = (double)(now - wallTimeOrigin_) * (double)oldSpeed;
        wallTimeOrigin_ = now - (int64_t)(currentFileOffset / (double)playbackSpeed_);
    }
    ofLog(OF_LOG_NOTICE, "Playback speed: %.2fx (slider=%.2f)", playbackSpeed_, sliderPos);
}

//--------------------------------------------------------------
float ofxDVS::getPlaybackSpeed(){
    return playbackSpeed_;
}

//--------------------------------------------------------------
void ofxDVS::resetPlaybackTiming(){
    timingInitialized_ = false;
    fileTimeOrigin_ = 0;
    wallTimeOrigin_ = 0;
    fileTimePaused_ = 0;
}

//--------------------------------------------------------------
void ofxDVS::changePause(){
    if(paused){
        // Resuming: recalculate wall origin so playback continues seamlessly
        paused = false;
        if (timingInitialized_ && playbackSpeed_ > 0.0f) {
            wallTimeOrigin_ = ofGetElapsedTimeMicros() -
                              (int64_t)((double)fileTimePaused_ / (double)playbackSpeed_);
        }
        thread.lock();
        thread.paused = paused;
        thread.unlock();
    }else{
        // Pausing: save current file-time progress
        if (timingInitialized_) {
            int64_t wallElapsed = ofGetElapsedTimeMicros() - wallTimeOrigin_;
            fileTimePaused_ = (int64_t)((double)wallElapsed * (double)playbackSpeed_);
        }
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
    }
}


void ofxDVS::onToggleEvent(ofxDatGuiToggleEvent e)
{
    if (e.target->is("ENABLE TRACKER")) {
        auto checked = e.target->getChecked();
        this->tracker_panel->setVisible(checked);
        this->enableTracker(checked);
        ofLog(OF_LOG_NOTICE, "Tracker enabled %d", checked);
    }else if(e.target->getLabel() == "APS"){
    	changeAps();
    }else if(e.target->getLabel() == "DVS"){
    	changeDvs();
    }else if(e.target->getLabel() == "IMU"){
    	changeImu();
    }else if (e.target->is("ENABLE OPTICAL FLOW")) {
        bool checked = e.target->getChecked();
        if (this->optflow_panel) this->optflow_panel->setVisible(checked);
        ofLog(OF_LOG_NOTICE, "Optical Flow panel %s", checked ? "shown" : "hidden");
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
    }else if (e.target->is("HOT PIXEL FILTER")) {
        hotPixelFilterEnabled_ = e.target->getChecked();
        ofLogNotice() << "Hot pixel filter " << (hotPixelFilterEnabled_ ? "enabled" : "disabled");
    }else if (e.target->is("BA FILTER")) {
        baFilterEnabled_ = e.target->getChecked();
        ofLogNotice() << "BA filter " << (baFilterEnabled_ ? "enabled" : "disabled");
    }else if (e.target->is("ENABLE NEURAL NETS")) {
        nnEnabled = e.target->getChecked();
        if (!nnEnabled) yolo_pipeline.clearHistory();
        ofLog(OF_LOG_NOTICE, "Neural nets %s", nnEnabled ? "enabled" : "disabled");
    }

}

void ofxDVS::onSliderEvent(ofxDatGuiSliderEvent e)
{
    if(e.target->getLabel() == "Playback Speed"){
        setPlaybackSpeed(e.value);
        float spd = std::pow(10.0f, e.value);
        char buf[16];
        if (spd < 1.0f) snprintf(buf, sizeof(buf), "%.2fx", spd);
        else if (spd < 10.0f) snprintf(buf, sizeof(buf), "%.1fx", spd);
        else snprintf(buf, sizeof(buf), "%dx", (int)spd);
        if (mySpeedDisplay) mySpeedDisplay->setText(buf);
    }else if(e.target->getLabel() == "DVS Integration"){
        cout << "Integration fsint is : " << e.value << endl;
        changeFSInt(e.value);
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
    }

    myCam.reset(); // no mesh turning when using GUI
}

void ofxDVS::recalibrateHotPixels() {
    hot_calib_done_ = false;
    hot_calib_started_ = false;
    hot_calib_start_ts_ = 0;
    std::fill(hot_calib_count_.begin(), hot_calib_count_.end(), 0);
    std::fill(hot_pixel_mask_.begin(), hot_pixel_mask_.end(), false);
    ofLogNotice() << "[HotPixel] Calibration restarted";
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
    if (!hotPixelFilterEnabled_) return;
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
        // Only filter when timestamps move forward; backward jump (file loop)
        // means new data that should be accepted.
        int64_t last = last_ts_map_[idx];
        int64_t dt = e.timestamp - last;
        if (last != 0 && dt >= 0 && dt < hot_refrac_us) {
            e.valid = false;
            continue;
        }
        last_ts_map_[idx] = e.timestamp;

        // --- rate-based check ---
        // reset window on rollover or backward timestamp jump (file loop)
        if (hot_rate_window_start_ == 0 || e.timestamp < hot_rate_window_start_) {
            std::fill(hot_rate_count_.begin(), hot_rate_count_.end(), 0);
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
