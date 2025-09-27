//
//  ofxDVS.cpp
//  ofxDVS
//
//  Created by Federico Corradi on 19.05.17.
//  updated since then till 2024
//
//

#include "ofxDVS.hpp"

#include <deque>

using namespace std::placeholders;
static constexpr float VIEW_SCALE = 6.0f / 7;

// yolo

#include <algorithm>
#include <tuple>

/*static const char* COCO80[80] = {
"person","bicycle","car","motorcycle","airplane","bus","train","truck","boat","traffic light",
"fire hydrant","stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow",
"elephant","bear","zebra","giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee",
"skis","snowboard","sports ball","kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle",
"wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich","orange",
"broccoli","carrot","hot dog","pizza","donut","cake","chair","couch","potted plant","bed",
"dining table","toilet","tv","laptop","mouse","remote","keyboard","cell phone","microwave","oven",
"toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"
};*/

static constexpr int yolo_num_classes = 1;
static const char* PEDRO_CLASSES[yolo_num_classes] = { "person"}; 

namespace {
// Nearest-neighbor letterbox for CHW floats: (C,Hs,Ws) -> (C,Hd,Wd)
std::vector<float> letterboxCHW(const std::vector<float>& src,
                                int C, int Hs, int Ws,
                                int Hd, int Wd,
                                float &scale, int &padx, int &pady)
{
    std::vector<float> dst((size_t)C * Hd * Wd, 0.0f);
    scale = std::min((float)Wd / (float)Ws, (float)Hd / (float)Hs);
    const int newW = (int)std::round(Ws * scale);
    const int newH = (int)std::round(Hs * scale);
    padx = (Wd - newW) / 2;
    pady = (Hd - newH) / 2;

    auto atSrc = [&](int c, int y, int x) -> float {
        return src[(size_t)c * Hs * Ws + (size_t)y * Ws + x];
    };
    auto atDst = [&](int c, int y, int x) -> float& {
        return dst[(size_t)c * Hd * Wd + (size_t)y * Wd + x];
    };

    for (int y = 0; y < newH; ++y) {
        int sy = std::min(Hs - 1, (int)std::floor(y / scale));
        for (int x = 0; x < newW; ++x) {
            int sx = std::min(Ws - 1, (int)std::floor(x / scale));
            for (int c = 0; c < C; ++c) {
                atDst(c, y + pady, x + padx) = atSrc(c, sy, sx);
            }
        }
    }
    return dst;
}

inline ofRectangle unletterboxAndProject(
    float x1, float y1, float x2, float y2,
    float scale, int padx, int pady,
    int sensorW, int sensorH, float drawW, float drawH)
{
    // model -> sensor
    float sx1 = (x1 - padx) / scale;
    float sy1 = (y1 - pady) / scale;
    float sx2 = (x2 - padx) / scale;
    float sy2 = (y2 - pady) / scale;

    // clamp to sensor bounds
    sx1 = std::max(0.f, std::min((float)sensorW, sx1));
    sy1 = std::max(0.f, std::min((float)sensorH, sy1));
    sx2 = std::max(0.f, std::min((float)sensorW, sx2));
    sy2 = std::max(0.f, std::min((float)sensorH, sy2));

    // sensor -> screen
    const float Sx = drawW / (float)sensorW;
    const float Sy = drawH / (float)sensorH;
    float x = sx1 * Sx;
    float y = sy1 * Sy;
    float w = (sx2 - sx1) * Sx;
    float h = (sy2 - sy1) * Sy;
    return ofRectangle(x, y, w, h);
}

// IoU for screen-space rectangles
static float rectIoU(const ofRectangle& A, const ofRectangle& B) {
    float ax1=A.getX(), ay1=A.getY(), ax2=ax1+A.getWidth(),  ay2=ay1+A.getHeight();
    float bx1=B.getX(), by1=B.getY(), bx2=bx1+B.getWidth(),  by2=by1+B.getHeight();
    float x1 = std::max(ax1, bx1), y1 = std::max(ay1, by1);
    float x2 = std::min(ax2, bx2), y2 = std::min(ay2, by2);
    float w = std::max(0.f, x2 - x1), h = std::max(0.f, y2 - y1);
    float inter = w*h;
    float aA = std::max(0.f, A.getWidth())  * std::max(0.f, A.getHeight());
    float aB = std::max(0.f, B.getWidth())  * std::max(0.f, B.getHeight());
    float denom = aA + aB - inter;
    return denom > 0.f ? inter / denom : 0.f;
}

// Average current-frame boxes with matches from the last 2 frames
static std::vector<ofxDVS::YoloDet>
temporalSmooth3(const std::vector<ofxDVS::YoloDet>& cur,
                std::deque<std::vector<ofxDVS::YoloDet>>& hist,
                int max_hist = 3, float match_iou = 0.5f,
                int min_hits = 2, float min_w = 12.f, float min_h = 12.f)
{
    // push current into history (screen coords)
    hist.push_back(cur);
    if ((int)hist.size() > max_hist) hist.pop_front();

    std::vector<ofxDVS::YoloDet> out;
    out.reserve(cur.size());

    for (const auto& d0 : cur) {
        if (d0.box.getWidth() < min_w || d0.box.getHeight() < min_h) continue;

        float sum_w = d0.score;
        float x1 = d0.box.getX() * d0.score;
        float y1 = d0.box.getY() * d0.score;
        float x2 = (d0.box.getX() + d0.box.getWidth())  * d0.score;
        float y2 = (d0.box.getY() + d0.box.getHeight()) * d0.score;
        int   hits = 1;

        for (int t = (int)hist.size() - 2; t >= 0; --t) {
            const auto& prev = hist[t];
            int best_j = -1; float best_iou = 0.f; float best_s = 0.f; ofRectangle best_r;
            for (int j = 0; j < (int)prev.size(); ++j) {
                if (prev[j].cls != d0.cls) continue;
                float iou = rectIoU(d0.box, prev[j].box);
                if (iou > best_iou) { best_iou = iou; best_j = j; best_s = prev[j].score; best_r = prev[j].box; }
            }
            if (best_j >= 0 && best_iou >= match_iou) {
                if (best_r.getWidth() >= min_w && best_r.getHeight() >= min_h) {
                    x1 += best_r.getX() * best_s;
                    y1 += best_r.getY() * best_s;
                    x2 += (best_r.getX() + best_r.getWidth())  * best_s;
                    y2 += (best_r.getY() + best_r.getHeight()) * best_s;
                    sum_w += best_s;
                    ++hits;
                }
            }
        }

        if (hits >= min_hits) {
            float ax1 = x1 / sum_w, ay1 = y1 / sum_w;
            float ax2 = x2 / sum_w, ay2 = y2 / sum_w;
            ofRectangle r(ax1, ay1, ax2 - ax1, ay2 - ay1);
            if (r.getWidth() >= min_w && r.getHeight() >= min_h) {
                ofxDVS::YoloDet sm{ r, sum_w / hits, d0.cls };
                out.push_back(sm);
            }
        }
    }
    return out;
}

} // anonymous namespace


namespace {
struct Det { float x1,y1,x2,y2,score; int cls; };

static float IoU(const Det& a, const Det& b){
    float xx1 = std::max(a.x1, b.x1);
    float yy1 = std::max(a.y1, b.y1);
    float xx2 = std::min(a.x2, b.x2);
    float yy2 = std::min(a.y2, b.y2);
    float w = std::max(0.0f, xx2 - xx1);
    float h = std::max(0.0f, yy2 - yy1);
    float inter = w * h;
    float areaA = std::max(0.0f, a.x2 - a.x1) * std::max(0.0f, a.y2 - a.y1);
    float areaB = std::max(0.0f, b.x2 - b.x1) * std::max(0.0f, b.y2 - b.y1);
    return inter / std::max(1e-6f, areaA + areaB - inter);
}

static std::vector<Det> nms(std::vector<Det> dets, float iou_thresh){
    std::vector<Det> keep;
    std::sort(dets.begin(), dets.end(), [](auto& a, auto& b){ return a.score > b.score; });
    std::vector<char> removed(dets.size(), 0);
    for (size_t i = 0; i < dets.size(); ++i){
        if (removed[i]) continue;
        keep.push_back(dets[i]);
        for (size_t j = i + 1; j < dets.size(); ++j){
            if (!removed[j] && IoU(dets[i], dets[j]) > iou_thresh) removed[j] = 1;
        }
    }
    return keep;
}
} // namespace



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
    /*thread.getPocoThread().setPriority(Poco::Thread::PRIO_HIGHEST);
    cout << thread.getPocoThread().getPriority() << " min"
    	<<thread.getPocoThread().getMinOSPriority() <<
		" max" << thread.getPocoThread().getMaxOSPriority() << endl;
     */

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
        //cout << ns << endl;
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
    /*thread_alpha.lock();
    thread.lock();
    thread_alpha.sizeX = thread.sizeX;
    thread_alpha.sizeY = thread.sizeY;
    thread_alpha.fsint = fsint = 2;
    thread_alpha.init = true;
    thread.unlock();
    thread_alpha.unlock();*/

	// init framebuffer
    ofSetVerticalSync(true);
    ofSetBackgroundColor(255);
    
    // allocate fbo , mesh and imagePol
    fbo.allocate(sizeX, sizeY, GL_RGBA32F);
    tex = &fbo.getTexture();
    mesh.setMode(OF_PRIMITIVE_POINTS); //GL_POINT_SMOOTH
    //glEnable(GL_POINT_SMOOTH); // use circular points instead of square points


    // shader for points
    if (ofIsGLProgrammableRenderer()) {
        glEnable(GL_PROGRAM_POINT_SIZE);     // allow shader to set gl_PointSize
        // --- Vertex shader (GL3) ---
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
                gl_PointSize = uPointSize;   // pixels
            }
        )";
        // --- Fragment shader (GL3) ---
        const char* fsrc = R"(
            #version 150
            in vec4 vColor;
            out vec4 outputColor;
            void main() {
                outputColor = vColor;        // square point (no smoothing)
            }
        )";
        pointShader.setupShaderFromSource(GL_VERTEX_SHADER,   vsrc);
        pointShader.setupShaderFromSource(GL_FRAGMENT_SHADER, fsrc);
        pointShader.bindDefaults();
        pointShader.linkProgram();
    } else {
        // Legacy pipeline
        glDisable(GL_POINT_SMOOTH);          // keep points square
        glPointSize(pointSizePx);            // will be overridden if a shader is active
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
    //
    /*timestampcurrentelapsedlive = 0;
    timestampcurrentelapsedfile = 0;
    timestampcurrentelapsedfile_0 = 0;
    timestampcurrentelapsedfile_1 = 0;
    timestampcurrentelapsedlive_buf = 0;
    timestampcurrentelapsedfile_buf = 0;
    tip = true;*/
    
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
    f1->addSlider("BA Filter dt", 1, 100000, BAdeltaT);
    f1->addSlider("DVS Integration", 1, 100, fsint);
    f1->addSlider("DVS Image Gen", 1, 20000, numSpikes);
    f1->addToggle("ENABLE TRACKER", false);
    //myIMU = f1->addValuePlotter("IMU", 0, 1);
    f1->setPosition(x, y);
    f1->expand();
    f1->onButtonEvent(this, &ofxDVS::onButtonEvent);
    f1->onToggleEvent(this, &ofxDVS::onToggleEvent);
    f1->onSliderEvent(this, &ofxDVS::onSliderEvent);
    f1->onMatrixEvent(this, &ofxDVS::onMatrixEvent);
    f1->onTextInputEvent(this, &ofxDVS::onTextInputEvent);
    // neural net
    f1->addButton("Enable NN");  // toggles model execution
    f1->addToggle("Draw YOLO", true);
    f1->addSlider("YOLO Conf", 0.0, 1.0, yolo_conf_thresh);
    f1->addSlider("VTEI Window (ms)", 5, 200, vtei_win_ms);

    numPaused = 0;
    numPausedRec = 0;

    // tracker
    //f1 = new ofxDatGuiFolder("Control", ofColor::fromHex(0xFFD00B));
    this->tracker_panel.reset( new ofxDatGui( ofxDatGuiAnchor::TOP_RIGHT ) );
    auto tracker_folder = this->tracker_panel->addFolder(">> Tracker Controls" );
    this->tracker_panel->setVisible(true);

    auto& cfg = this->rectangularClusterTrackerConfig;
    tracker_folder->addToggle("FILTER", cfg.filterEnabled);
    tracker_folder->addSlider("UPDATE INTERVAL ms", 0, 1000, cfg.updateIntervalMs);
    tracker_folder->addSlider("MAX NUM CLUSTERS", 1, 100, cfg.maxNumClusters);
    tracker_folder->addBreak()->setHeight(5.0f);
    // Display
    tracker_folder->addToggle("ELLIPTICAL CLUSTERS", cfg.useEllipticalClusters);
    tracker_folder->addSlider("PATH LENGTH", 1, 500, cfg.pathLength);
    tracker_folder->addToggle("SHOW CLUSTER NUMBER", cfg.showClusterNumber);
    tracker_folder->addToggle("SHOW CLUSTER EPS", cfg.showClusterEps);
    tracker_folder->addToggle("SHOW CLUSTER RADIUS", cfg.showClusterRadius);
    tracker_folder->addToggle("SHOW CLUSTER VELOCITY", cfg.showClusterVelocity);
    tracker_folder->addToggle("SHOW CLUSTER VEL VECTOR", cfg.showClusterVelocityVector);
    tracker_folder->addToggle("SHOW CLUSTER MASS", cfg.showClusterMass);
    tracker_folder->addToggle("SHOW PATHS", cfg.showPaths);
    tracker_folder->addSlider("VELOCITY VECTOR SCALING", 0, 10, cfg.velocityVectorScaling);
    tracker_folder->addBreak()->setHeight(5.0f);
    // Movement
    tracker_folder->addSlider("MIXING FACTOR", 0, 1, cfg.mixingFactor);
    tracker_folder->addToggle("PATHS", cfg.pathsEnabled);
    tracker_folder->addToggle("USE VELOCITY", cfg.useVelocity);
    tracker_folder->addToggle("USE NEAREST CLUSTER", cfg.useNearestCluster);
    tracker_folder->addSlider("PREDICTIVE VELOCITY", 0, 100, cfg.predictiveVelocityFactor);
    tracker_folder->addToggle("initializeVelocityToAverage", cfg.initializeVelocityToAverage);
    tracker_folder->addSlider("VELOCITY TAU ms", 0, 1000, cfg.velocityTauMs);
    tracker_folder->addSlider("FRICTION TAU ms", 0, 1000, cfg.frictionTauMs);
    tracker_folder->addBreak()->setHeight(5.0f);
    // Sizing
    tracker_folder->addSlider("SURROUND", 0, 10, cfg.surround);
    tracker_folder->addToggle("DYNAMIC SIZE", cfg.dynamicSizeEnabled);
    tracker_folder->addToggle("DYNAMIC ASPECT RATIO", cfg.dynamicAspectRatioEnabled);
    tracker_folder->addToggle("DYNAMIC ANGLE", cfg.dynamicAngleEnabled);
    tracker_folder->addSlider("ASPECT RATIO", 0, 2, cfg.aspectRatio);
    tracker_folder->addSlider("CLUSTER SIZE", 0, 2, cfg.clusterSize);
    tracker_folder->addToggle("HIGHWAY PERSPECTIVE", cfg.highwayPerspectiveEnabled);
    tracker_folder->addToggle("ANGLE FOLLOWS VELOCITY", cfg.angleFollowsVelocity);
    tracker_folder->addBreak()->setHeight(5.0f);
    // Update
    tracker_folder->addToggle("ONE POLARITY", cfg.useOnePolarityOnlyEnabled);
    tracker_folder->addToggle("GROW MERGED SIZE", cfg.growMergedSizeEnabled);
    tracker_folder->addSlider("velAngDiffDegToNotMerge", 0, 360, cfg.velAngDiffDegToNotMerge);
    tracker_folder->addBreak()->setHeight(5.0f);
    // Lifetime
    tracker_folder->addSlider("THRESHOLD MASS", 0, 100, cfg.thresholdMassForVisibleCluster);
    tracker_folder->addSlider("THRESHOLD VELOCITY", 0, 100, cfg.thresholdVelocityForVisibleCluster);
    tracker_folder->addSlider("MASS DECAY TAU us", 0, 100000, cfg.clusterMassDecayTauUs);
    tracker_folder->addToggle("CLUSTER EXIT PURGING", cfg.enableClusterExitPurging);
    tracker_folder->addToggle("SURROUND INHIBITION", cfg.surroundInhibitionEnabled);
    tracker_folder->addSlider("SURROUND INHIBITION COST", 0, 10, cfg.surroundInhibitionCost);
    tracker_folder->addToggle("DO NOT MERGE", cfg.dontMergeEver);
    tracker_folder->addBreak()->setHeight(5.0f);
    // PI Controller
    tracker_folder->addToggle("SMOOTH MOVE", cfg.smoothMove);
    tracker_folder->addSlider("SMOOTH WEIGHT", 0, 1000, cfg.smoothWeight);
    tracker_folder->addSlider("SMOOTH POSITION", 0, 0.1f, cfg.smoothPosition);
    tracker_folder->addSlider("SMOOTH INTEGRAL", 0, 0.1f, cfg.smoothIntegral);

    this->tracker_panel->onToggleEvent(this, &ofxDVS::onTrackerToggleEvent);         // toggle press
    this->tracker_panel->onSliderEvent(this, &ofxDVS::onTrackerSliderEvent);         // slide move

    /* 2d visualisation primitives setup
     */

    this->next_polarities_pixbuf.allocate(this->sizeX, this->sizeY, OF_IMAGE_COLOR);
    this->next_polarities.allocate (this->next_polarities_pixbuf);
    this->next_frame.allocate(this->next_polarities_pixbuf);
    //this->next_polarities.allocate (this->dvs_frame_W, this->dvs_frame_H, GL_RGB);
    //this->next_frame.allocate(this->dvs_frame_W, this->dvs_frame_H, GL_RGB);
    updateViewports();


    /* 3d visualisation primitives setup
     */

    this->next_polarities_3d.setMode(OF_PRIMITIVE_POINTS);
    this->next_polarities_3d.enableColors();

    // DepthTest will make sure that 3d elements that are covered by other elements, won’t be rendered
    ofEnableDepthTest();  // generally good, but sometimes problematic with 2D primitives
    ofDisableDepthTest();

    // use circular points instead of square points
    //glEnable(GL_POINT_SMOOTH);

    // set the mesh points size
    glPointSize(1);

    // onnx runner
    try {
        OnnxRunner::Config nncfg;
        nncfg.model_path = ofToDataPath("model.onnx", true); // adjust path
        nncfg.intra_op_num_threads = 1;
        nncfg.normalize_01 = true;
        nncfg.verbose = false;
        nn = std::make_unique<OnnxRunner>(nncfg);
        nn->load();
        ofLogNotice() << "NN loaded.";
    } catch (const std::exception& e) {
        ofLogError() << "Failed to load NN: " << e.what();
    }

    // hot pixels
    last_ts_map_.assign(this->sizeX * this->sizeY, 0);

}

void ofxDVS::updateViewports()
{
    this->frame_viewport = ofRectangle(20, 20, this->next_frame.getWidth()* VIEW_SCALE, this->next_frame.getHeight() * VIEW_SCALE);
    this->polarities_viewport = ofRectangle(350, 20, this->next_polarities.getWidth()* VIEW_SCALE, this->next_polarities.getHeight()* VIEW_SCALE);
    this->cam_viewport = ofRectangle(700.0, 20, this->sizeX, this->sizeY);
}

void ofxDVS::onTrackerSliderEvent(ofxDatGuiSliderEvent e) {
    auto& cfg = this->rectangularClusterTrackerConfig;
    static std::map<std::string, float*> vars = {
        { "UPDATE INTERVAL ms",     &cfg.updateIntervalMs },
        { "MAX NUM CLUSTERS",       &cfg.maxNumClusters },
        { "PATH LENGTH",            &cfg.pathLength },
        { "VELOCITY VECTOR SCALING", &cfg.velocityVectorScaling },
        { "MIXING FACTOR",          &cfg.mixingFactor },
        { "PREDICTIVE VELOCITY",    &cfg.predictiveVelocityFactor },
        { "VELOCITY TAU ms",        &cfg.velocityTauMs },
        { "FRICTION TAU ms",        &cfg.frictionTauMs },
        { "SURROUND",               &cfg.surround },
        { "ASPECT RATIO",           &cfg.aspectRatio },
        { "CLUSTER SIZE",           &cfg.clusterSize },
        { "velAngDiffDegToNotMerge", &cfg.velAngDiffDegToNotMerge },
        { "THRESHOLD MASS",         &cfg.thresholdMassForVisibleCluster },
        { "THRESHOLD VELOCITY",     &cfg.thresholdVelocityForVisibleCluster },
        { "MASS DECAY TAU us",      &cfg.clusterMassDecayTauUs },
        { "SURROUND INHIBITION COST", &cfg.surroundInhibitionCost },
        { "SMOOTH WEIGHT",          &cfg.smoothWeight },
        { "SMOOTH POSITION",        &cfg.smoothPosition },
        { "SMOOTH INTEGRAL",        &cfg.smoothIntegral },
    };

    auto it = vars.find(e.target->getName());
    if (it != vars.end()) {
        *(it->second) = e.target->getValue();
    }
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


// ---------------------------------------------------------------------------
void ofxDVS::onTrackerToggleEvent(ofxDatGuiToggleEvent e) {
    auto& cfg = this->rectangularClusterTrackerConfig;
    static std::map<std::string, bool*> vars = {
        { "FILTER",                 &cfg.filterEnabled },
        { "ELLIPTICAL CLUSTERS",    &cfg.useEllipticalClusters },
        { "SHOW CLUSTER NUMBER",    &cfg.showClusterNumber },
        { "SHOW CLUSTER EPS",       &cfg.showClusterEps },
        { "SHOW CLUSTER RADIUS",    &cfg.showClusterRadius },
        { "SHOW CLUSTER VELOCITY",  &cfg.showClusterVelocity },
        { "SHOW CLUSTER VEL VECTOR", &cfg.showClusterVelocityVector },
        { "SHOW CLUSTER MASS",      &cfg.showClusterMass },
        { "SHOW PATHS",             &cfg.showPaths },
        { "PATHS",                  &cfg.pathsEnabled },
        { "USE VELOCITY",           &cfg.useVelocity },
        { "USE NEAREST CLUSTER",    &cfg.useNearestCluster },
        { "initializeVelocityToAverage", &cfg.initializeVelocityToAverage },
        { "DYNAMIC SIZE",           &cfg.dynamicSizeEnabled },
        { "DYNAMIC ASPECT RATIO",   &cfg.dynamicAspectRatioEnabled },
        { "DYNAMIC ANGLE",          &cfg.dynamicAngleEnabled },
        { "HIGHWAY PERSPECTIVE",    &cfg.highwayPerspectiveEnabled },
        { "ANGLE FOLLOWS VELOCITY", &cfg.angleFollowsVelocity },
        { "ONE POLARITY",           &cfg.useOnePolarityOnlyEnabled },
        { "GROW MERGED SIZE",       &cfg.growMergedSizeEnabled },
        { "GROW MERGED SIZE",       &cfg.growMergedSizeEnabled },
        { "CLUSTER EXIT PURGING",   &cfg.enableClusterExitPurging },
        { "SURROUND INHIBITION",    &cfg.surroundInhibitionEnabled },
        { "DO NOT MERGE",           &cfg.dontMergeEver },
        { "SMOOTH MOVE",            &cfg.smoothMove },
    };

    auto it = vars.find(e.target->getName());
    if (it != vars.end()) {
        *(it->second) = e.target->getChecked();
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
    //string path;
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
    //string path;
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

	// every time here get targetSpeed us of data (fixed us playback)
	//long timeInterval = targetSpeed;
    if (packetContainer == NULL) {
        return(false); // Skip if nothing there.
    }
    
    int32_t packetNum = caerEventPacketContainerGetEventPacketsNumber(packetContainer);
    /*
    firstTs = caerEventPacketContainerGetLowestEventTimestamp(packetContainer);
    highestTs = caerEventPacketContainerGetHighestEventTimestamp(packetContainer);
	long current_file_dt = highestTs-firstTs;
	long current_ofx_dt = ofGetElapsedTimeMicros() - ofxLastTs;

	cout << "highestTs " << highestTs << " FirstTS " << firstTs << " ofxLastTs " << ofxLastTs << endl;
	cout << " current_file_dt " <<  current_file_dt << " current_ofx_dt " << current_ofx_dt << endl;
     */

    for (int32_t i = 0; i < packetNum; i++) {
        
        polarity nuPack;
        frame nuPackFrames;
        imu6 nuPackImu6;
        
        caerEventPacketHeader packetHeader = caerEventPacketContainerGetEventPacket(packetContainer, i);
        if (packetHeader == NULL) {
            //ofLog(OF_LOG_WARNING,"Packet %d is empty (not present).\n", i);
            continue; // Skip if nothing there.
        }
        
        int type = caerEventPacketHeaderGetEventType(packetHeader);
        long lastTs = 0;

        //ofLog(OF_LOG_WARNING,"Packet %d of type %d -> size is %d.\n", i, caerEventPacketHeaderGetEventType(packetHeader),
        //           caerEventPacketHeaderGetEventNumber(packetHeader));
        
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
            // first time we get in here
            // otherwise we do not clear packetframes
            // and we keep the last one
            
            packetsFrames.clear();
            packetsFrames.shrink_to_fit();

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
        thread.lock();

        // Detect resolution changes (live , file, or different file)
        if ((thread.deviceReady || thread.fileInputReady) &&
            (thread.sizeX != sizeX || thread.sizeY != sizeY)) {

            sizeX = thread.sizeX;
            sizeY = thread.sizeY;

            // Recreate the tracker at the new resolution
            if (rectangularClusterTrackerEnabled) {
                createRectangularClusterTracker();
            }

            // (Optional but recommended) keep your viewports in sync
            updateViewports();

            // (Optional) if you rely on buffers sized to sizeX/sizeY
            // reallocate imagePolarity, visualizerMap, etc., here.
        }


        for(size_t i=0; i<thread.container.size(); i++){
            packetContainer = thread.container[i];

            long firstTs = caerEventPacketContainerGetLowestEventTimestamp(packetContainer);
            long highestTs = caerEventPacketContainerGetHighestEventTimestamp(packetContainer);
            long current_file_dt = highestTs-firstTs;
            long current_ofx_dt = ofGetElapsedTimeMicros() - ofxLastTs; // application started - last update
            //ofLog(OF_LOG_NOTICE, "current_ofx_dt %04lu", current_ofx_dt*100);
            bool delpc = false;
            if(targetSpeed <= 0){
            	targetSpeed = 0.0000001;
            }
            //is live of file faster?
            //ofLog(OF_LOG_NOTICE, "file dt %04lu", current_file_dt);

            if( current_file_dt <  (float)current_ofx_dt/targetSpeed){
                //cout << "highestTs " << highestTs << " FirstTS " << firstTs << " ofxLastTs " << ofxLastTs << endl;
                //cout << " current_file_dt " <<  current_file_dt << " current_ofx_dt " << current_ofx_dt << endl;
                delpc = organizeData(packetContainer);
                //nanosleep((const struct timespec[]){{0, 5000000L}}, NULL);
                
                // update time information
                long ts = caerEventPacketContainerGetHighestEventTimestamp(packetContainer);
                //ofLog(OF_LOG_NOTICE, "timestamp %04lu", ts);
                // just start
                if(ts != -1){
                    if(isStarted == false){
                        started = ts;
                       // timestampcurrentelapsedfile = 0;
                       // timestampcurrentelapsedlive_buf = 0;
                       // timestampcurrentelapsedfile_buf = 0;
                        isStarted = true;
                    }
                    // we restart
                    if(ts < started){
                        started = ts;
                       // timestampcurrentelapsedfile = 0;
                       // timestampcurrentelapsedlive_buf = 0;
                       // timestampcurrentelapsedfile_buf = 0;
                    }
                    unsigned long current =  ts - started;
                    microseconds = current - (minutes*60)*1e6 - seconds*1e6;
                    minutes = (current / 60e6);
                    seconds = ( ((int)current % (int)60e6) / 1e6);
                    hours =  ((((int)current % (int)60e6) / 1e6) / 1e6);
                    //cout << hours << ":" << minutes << ":" << seconds << ":" << microseconds << endl;
                    sprintf(timeString, " %02lu:%02lu:%02lu:%04lu", hours, minutes, seconds, microseconds);
                }else{
                    unsigned long zero = 0;
                    sprintf(timeString, "%02lu", zero);
                }
            }else{
                // no deal
                if(i>0){
                    i = i-1;
                }
            }// target speed
            // recording status
            if(isRecording){
                // order packet containers in time - file format aedat 3.1 standard -
                size_t currPacketContainerSize = (size_t) caerEventPacketContainerGetEventPacketsNumber(packetContainer);
                qsort(packetContainer->eventPackets, currPacketContainerSize, sizeof(caerEventPacketHeader),
                      &packetsFirstTimestampThenTypeCmp);
                // save to files
                int32_t packetNum = caerEventPacketContainerGetEventPacketsNumber(packetContainer);
                for (int32_t i = 0; i < packetNum; i++) {
                    caerEventPacketHeader packetHeader = caerEventPacketContainerGetEventPacket(packetContainer, i);
                    if(packetHeader == NULL){
                        continue;
                    }
                    caerEventPacketHeaderSetEventCapacity(packetHeader, caerEventPacketHeaderGetEventNumber(packetHeader));
                    size_t sizePacket = caerEventPacketGetSize(packetHeader);
                    caerEventPacketHeaderSetEventSource(packetHeader, caerEventPacketHeaderGetEventSource(packetHeader));
                    myFile.write((char*)packetHeader, sizePacket);
                }
            }
            // free all packet containers here
            if(delpc){
            	caerEventPacketContainerFree(packetContainer);
            	thread.container.erase( thread.container.begin()+i );
            }
            thread.doLoad = delpc;
        }
        //thread.container.clear();
        //thread.container.shrink_to_fit();
        // done with the resource
        thread.unlock();
        
        // check how fast we are going, if we are too slow, drop some data    
    }else{
        // we are paused..
        thread.lock();
        // in live mode .. just trow away
        for(size_t i=0; i<thread.container.size(); i++){
            packetContainer = thread.container.back(); // this is a pointer
            thread.container.pop_back();
            // free all packet containers here
            caerEventPacketContainerFree(packetContainer);
        }
        // done with the resource
        thread.unlock();
    }
    

    updateBAFilter();
    applyRefractory_();
    updateImageGenerator();

    // --- FEED RECTANGULAR CLUSTER TRACKER ---
    if (rectangularClusterTrackerEnabled && rectangularClusterTracker) {
        RectangularClusterTracker::PolaritiesQueue inq, outq; // both std::deque
        inq.clear(); outq.clear();

        // Fill input queue from current polarity packet
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

        // Run tracker on this packet
        rectangularClusterTracker->filter(inq, outq);
        rectangularClusterTracker->updateClusterList(latest_ts); // <-- DO THIS
    }



    //GUI
    f1->update();
    myTextTimer->setText(timeString);
    //float val = ofRandom(0, 1);
    //cout << val << endl;
    //myIMU->setValue(val);
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
    /*thread_alpha.lock();
    thread_alpha.fsint  = fsint;
    thread_alpha.unlock();*/
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
			thisV.x = thisV.x-ofGetWidth()/2;     //ofTranslate(ofPoint(-ofGetWidth()/2,-ofGetHeight()/2));
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
		//ofSetColor(ofColor::gray);
		//ofDrawLine(nearestVertex, mouse);

		ofNoFill();
		//ofSetColor(ofColor::yellow);
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
    
    //myCam.begin();
    //ofTranslate(ofPoint(-1024,1024));
    //drawFrames();
    //drawImageGenerator(); // if dvs.drawImageGen
    //drawImageGenerator();
    //updateMeshSpikes();   // first update the mesh
    //drawSpikes();         // then draw spikes - if dvs.doDrawSpikes
    //drawImu6();
    //myCam.setPosition(200, 40, 0);
    //myCam.end();
    
    //imageGenerator.draw(440,370,346,240);
    
    //if rotation and tranlation quaternions
    //  ofMatrix4x4 matrix = 0;
    //ofMatrix4x4 rotationCam;
    //myCam.reset();
    //myCam.setOrientation(rotationCam);
    //}
    //ofDrawBox(cameraPos.x*10, cameraPos.y*10, cameraPos.z*10, 20, 20, 20);
    //myCam.setTarget(cameraPos*120);
    //cout << "DVS class" << rotationCam << endl;
    //drawMouseDistanceToSpikes();
    
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

    // neutral stage (no extra transform inside)
    rectangularClusterTracker->draw(ofRectangle(0, 0, sizeX, sizeY));

    ofPopMatrix();
    ofPopStyle();
}

void ofxDVS::drawYoloDetections()
{
    if (!yolo_draw || yolo_dets.empty()) return;

    // ASSUMPTION: d.box is in CHIP PIXELS (x,y,w,h in [0..sizeX/sizeY]).
    // If your boxes are still in screen coords, convert them to chip coords first.

    ofPushStyle();
    ofDisableDepthTest();
    ofNoFill();
    ofSetColor(255, 215, 0);   // gold
    ofSetLineWidth(3.0f);

    ofPushMatrix();
    // chip -> full screen
    ofScale(ofGetWidth() / (float)sizeX, ofGetHeight() / (float)sizeY);
    ofScale(1.0f, -1.0f);
    ofTranslate(0.0f, -(float)sizeY);

    for (auto &d : yolo_dets) {
        ofDrawRectangle(d.box);

        // Label: flip just the text back so it’s readable
        std::string name = (d.cls>=0 && d.cls<yolo_num_classes)
                           ? PEDRO_CLASSES[d.cls]
                           : ("id:"+ofToString(d.cls));
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%s %.2f", name.c_str(), d.score);

        ofPushMatrix();
        ofTranslate(d.box.getX() + 2, d.box.getY() + d.box.getHeight() - 4);
        ofScale(1.0f, -1.0f);
        ofDrawBitmapStringHighlight(buf, 0, 0, ofColor(0,0,0,180), ofColor(255,215,0));
        ofPopMatrix();
    }

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
    drawYoloDetections();

    myCam.end();
    
     // Pointer + GUI
    drawMouseDistanceToSpikes();
    //GUI
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
            //thread_alpha.visualizerMap[x][y] = visualizerMap[x][y];
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
        //imagePolarity.clear();
        vector<polarity> packets = getPolarity();
        for(int i=0;i < packets.size();i++) {
            
            int x = (int)packetsPolarity[i].pos.x;
            int y = (int)packetsPolarity[i].pos.y;
            int alpha = 255;//(int)ceil(visualizerMap[x][y]);
            
            //long tdiff = (int) ofRandom(1000) % 1000;//packets[i].timestamp - dvs.ofxLastTs;
            long tdiff = 0;
            if( packets[i].timestamp < tmp){
                //ofLog(OF_LOG_NOTICE, "Detected lower timestamp.. ");
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
                tmp = packets[i].timestamp;//tmp-nus;
            }
            long timeus = 0;
            if(m == 0){
                timeus = 0;
            }else{
                timeus = tdiff>>m;
            }
            //mesh.addVertex(ofVec3f(ofMap(packets[i].pos.x,0,sizeX,0,ofGetWidth()),ofMap(packets[i].pos.y,sizeY,0,0,ofGetHeight()), timeus));
            mesh.addVertex(ofVec3f(
                ofMap(packets[i].pos.x, 0, sizeX, 0, ofGetWidth()),
                ofMap(packets[i].pos.y, 0, sizeY, 0, ofGetHeight()),   // <-- no flip
                timeus));

            mesh.addTexCoord(ofVec2f(packets[i].pos.x,packets[i].pos.y));
            //int alphaus = (int)ceil(((float)tdiff/(float)nus)*256);
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

    //}
}

//--------------------------------------------------------------
void ofxDVS::drawSpikes() {
    
    /*if(doDrawSpikes){
        ofPushMatrix();
        mesh.draw();
        ofPopMatrix();
    }*/
    if(doDrawSpikes){
        for (int i = 0; i < packetsPolarity.size(); i++) {
            int x =(int)packetsPolarity[i].pos.x;
            int y =(int)packetsPolarity[i].pos.y;
            if(packetsPolarity[i].valid){
                visualizerMap[x][y] += 65;
            }
            //thread_alpha.visualizerMap[x][y] = visualizerMap[x][y];
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
        //imagePolarity.clear();
        vector<polarity> packets = getPolarity();
        for(int i=0;i < packets.size();i++) {
            
            int x = (int)packetsPolarity[i].pos.x;
            int y = (int)packetsPolarity[i].pos.y;
            int alpha = 255;//(int)ceil(visualizerMap[x][y]);
            
            //long tdiff = (int) ofRandom(1000) % 1000;//packets[i].timestamp - dvs.ofxLastTs;
            long tdiff = 0;
            if( packets[i].timestamp < tmp){
                //ofLog(OF_LOG_NOTICE, "Detected lower timestamp.. ");
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
                tmp = packets[i].timestamp;//tmp-nus;
            }
            long timeus = 0;
            if(m == 0){
                timeus = 0;
            }else{
                timeus = tdiff>>m;
            }
            mesh.addVertex(ofVec3f(ofMap(packets[i].pos.x,0,sizeX,0,ofGetWidth()),ofMap(packets[i].pos.y,sizeY,0,0,ofGetHeight()), timeus));
            mesh.addTexCoord(ofVec2f(packets[i].pos.x,packets[i].pos.y));
            //int alphaus = (int)ceil(((float)tdiff/(float)nus)*256);
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
        //mesh.draw();
        // use shader
        if (ofIsGLProgrammableRenderer()) {
            pointShader.begin();
            pointShader.setUniform1f("uPointSize", 4); // e.g. 8–12 px
            mesh.draw();   // OF_PRIMITIVE_POINTS
            pointShader.end();
        } else {
            glDisable(GL_POINT_SMOOTH);     // square
            glPointSize(4);       // make sure it’s set right before draw
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
    
    //thread.lock();
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
    //thread.unlock();
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
void ofxDVS::exit() {
    
    // stop the thread
    thread.stopThread();
    if(isRecording){
        // close file
        myFile.close();
    }

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
    
    // draw last imagegenerator frame
    //ofEnableAlphaBlending();
    //ofDisableAlphaBlending();
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

			/*ofRotateX(compAngleX);
			ofRotateY(compAngleY);
			ofRotateZ(compAngleZ);
			ofFill();
			ofSetColor(ofColor::red);
			ofDrawBox(500);
			ofNoFill();
			ofSetColor(ofColor::black);
			ofDrawBox(500);*/

		}
		ofPopStyle();
		//fbo.begin();
		/*ofPushStyle();
		//ofSetColor(0,0,0);
		ofVec3f arrowTailPoint = ofVec3f(100,100,0);
		//ofVec3f arrowHeadPoint = ofVec3f(150,100,0);
		//ofDrawArrow(arrowTailPoint, arrowHeadPoint, 20.0);


			cout << "x :" << packetsImu6[i].gyro.x << " y :" << packetsImu6[i].gyro.y <<
			" z :" << packetsImu6[i].gyro.z << endl;
			 ofSetColor(0,0,0);
		   ofDrawArrow(arrowTailPoint,packetsImu6[i].gyro, 10.0);
			 ofSetColor(255,255,0);
		   ofDrawArrow(arrowTailPoint,packetsImu6[i].accel, 10.0);

		ofPopStyle();*/
		//fbo.end();
		//fbo.draw(0,0,ofGetWidth(),ofGetHeight());
		//packetsImu6.clear();
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

std::vector<float> ofxDVS::buildVTEI_(int W, int H)
{
    // Source (sensor) size
    const int srcW = sizeX;
    const int srcH = sizeY;

    // 1) Accumulate pos/neg event counts in last window (e.g. 50ms)
    std::vector<float> pos(srcW * srcH, 0.0f), neg(srcW * srcH, 0.0f);
    long latest_ts = 0;
    for (const auto &e : packetsPolarity) {
        if (!e.valid) continue;
        if (e.timestamp > latest_ts) latest_ts = e.timestamp;
    }
    const long win_us = vtei_win_us; // 50 ms window
    for (const auto &e : packetsPolarity) {
        if (!e.valid) continue;
        if (e.timestamp + win_us >= latest_ts) {
            int x = int(e.pos.x), y = int(e.pos.y);
            if ((unsigned)x < (unsigned)srcW && (unsigned)y < (unsigned)srcH) {
                if (e.pol) pos[y*srcW + x] += 1.0f; else neg[y*srcW + x] += 1.0f;
            }
        }
    }
    // simple normalization for counts
    const float count_scale = 5.0f; // tune to match training
    for (size_t i=0;i<pos.size();++i) {
        pos[i] = std::min(1.0f, pos[i] / count_scale);
        neg[i] = std::min(1.0f, neg[i] / count_scale);
    }

    // 2) Time surface (exponential decay of last timestamp)
    std::vector<float> T(srcW * srcH, 0.0f);
    if (surfaceMapLastTs) {
        const float tau_us = 5e5f; // 0.5s time constant — tune for your training
        for (int y=0;y<srcH;++y) {
            for (int x=0;x<srcW;++x) {
                float last = surfaceMapLastTs[y][x]; // expect microseconds
                float dt   = std::max(0.0f, float(latest_ts) - last);
                float val  = std::exp(-dt / tau_us);
                T[y*srcW + x] = std::clamp(val, 0.0f, 1.0f);
            }
        }
    }

    // 3) Intensity (I) and simple edge magnitude (E) from imageGenerator (no OpenCV)
    ofImage I = imageGenerator; // make a local copy we can resize/convert
    I.setImageType(OF_IMAGE_GRAYSCALE);
    if (I.getWidth() != srcW || I.getHeight() != srcH) I.resize(srcW, srcH);

    std::vector<float> E(srcW * srcH, 0.0f);
    auto clamp01 = [](float v){ return std::max(0.0f, std::min(1.0f, v)); };
    const ofPixels &Ip = I.getPixels();
    auto P = [&](int yy, int xx)->unsigned char { return Ip[yy*srcW + xx]; };

    for (int y=1; y<srcH-1; ++y){
        for (int x=1; x<srcW-1; ++x){
            float gx = float(P(y-1,x+1) + 2*P(y,x+1) + P(y+1,x+1)
                           - P(y-1,x-1) - 2*P(y,x-1) - P(y+1,x-1));
            float gy = float(P(y+1,x-1) + 2*P(y+1,x) + P(y+1,x+1)
                           - P(y-1,x-1) - 2*P(y-1,x) - P(y-1,x+1));
            float mag = std::sqrt(gx*gx + gy*gy) / (4.0f * 255.0f);
            E[y*srcW + x] = clamp01(mag);
        }
    }

    // 4) Resize (nearest) to network size and pack to CHW (C=5: pos,neg,T,E,I)
    std::vector<float> chw(5 * W * H, 0.0f);

    auto nn_src_idx = [&](int yy, int xx)->int {
        int sx = std::min(srcW-1, std::max(0, int(std::round((xx + 0.5f) * srcW / float(W) - 0.5f))));
        int sy = std::min(srcH-1, std::max(0, int(std::round((yy + 0.5f) * srcH / float(H) - 0.5f))));
        return sy * srcW + sx;
    };

    for (int yy=0; yy<H; ++yy) {
        for (int xx=0; xx<W; ++xx) {
            size_t hw = size_t(yy) * W + xx;
            int sidx  = nn_src_idx(yy, xx);

            float i01 = Ip[sidx] / 255.0f; // intensity 0..1

            chw[0*(W*H) + hw] = pos[sidx];
            chw[1*(W*H) + hw] = neg[sidx];
            chw[2*(W*H) + hw] = T[sidx];
            chw[3*(W*H) + hw] = E[sidx];
            chw[4*(W*H) + hw] = i01;
        }
    }

    return chw;
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
    
    /*
    // Decay the map.
    for (size_t x = 0; x < sizeX; x++) {
        for (size_t y = 0; y < sizeY; y++) {
            
            if (surfaceMapLastTs[x][y] == 0) {
                continue;
            }
            else {
                
                spikeFeatures[x][y] -= decaySpikeFeatures; // Do decay.
                if (spikeFeatures[x][y] < 0) {
                    spikeFeatures[x][y] = 0;
                }
            }
        }
    }
    
    for (int col_idx = 0; col_idx < sizeX; col_idx++) {
        for (int row_idx = 0; row_idx < sizeY; row_idx++) {
            ofColor this_pixel;
            this_pixel.set(0,0,0,1);
            imageGenerator.setColor(col_idx,row_idx,this_pixel);
        }
    }
    
    for (int col_idx = 0; col_idx < sizeX; col_idx++) {
        for (int row_idx = 0; row_idx < sizeY; row_idx++) {
            
            ofColor this_pixel;
            float f = spikeFeatures[col_idx][row_idx]*255;
            if(f > 255){
                f = 255;
            }else if(f < 0){
                f = 0;
            }
            //ofLog(OF_LOG_NOTICE, "Float %f\n", f);
            this_pixel.set((int)floor(f), spkOnR[paletteSpike],spkOnG[paletteSpike], 125);
            imageGenerator.setColor(col_idx,row_idx,this_pixel);
        }
    }
    imageGenerator.update();*/

    if(numSpikes <= counterSpikes){

        counterSpikes = 0;
        // ofLog(OF_LOG_WARNING,"Generate Image \n");
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
            range = numSDevs * sig * (1.0f / 256.0f); //256 included here for nullhop reshift
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
                        f = 255; //255 included here for nullhop reshift
                    }else if (f < 0) {
                        f = 0;
                    }
                    ofColor this_pixel;
                    //spkOnR[paletteSpike],spkOnG[paletteSpike],spkOnB[paletteSpike]
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

        // After you update the image generator (e.g., inside update() after updateImageGenerator())
        newImageGen = true;

        if (nnEnabled && nn && nn->isLoaded() && newImageGen) {
            try {
                // 0) Sensor & draw sizes
                const int   sensorW = sizeX;                // e.g. 346 or 640
                const int   sensorH = sizeY;                // e.g. 260 or 480
                const float drawW   = (float)ofGetWidth();
                const float drawH   = (float)ofGetHeight();

                // 1) Model input size
                auto hw = nn->getInputHW();
                int Hd = hw.first;
                int Wd = hw.second;
                if (Hd <= 0 || Wd <= 0) { Hd = 288; Wd = 352; }  // PEDRo export size

                // 2) Build 5-ch VTEI at sensor size (C=5)
                const int C5 = 5;
                std::vector<float> chw5_sensor = buildVTEI_(sensorW, sensorH);

                // 3) Letterbox to model size
                float lb_scale = 1.f; int lb_padx = 0, lb_pady = 0;
                std::vector<float> chw5_model = letterboxCHW(
                    chw5_sensor, C5, /*Hs*/sensorH, /*Ws*/sensorW,
                    /*Hd*/Hd, /*Wd*/Wd, lb_scale, lb_padx, lb_pady
                );
                if (chw5_model.size() != (size_t)C5 * Hd * Wd) {
                    ofLogError() << "[YOLO] letterbox produced wrong size";
                    yolo_dets.clear();
                    return;
                }

                // 4) Run ONNX (direct CHW float)
                auto outmap = nn->runCHW(chw5_model, C5, Hd, Wd);
                if (outmap.empty()) { yolo_dets.clear(); return; }

                // Prefer "output0" explicitly; otherwise take first entry
                const std::vector<float>* pv = nullptr;
                auto it0 = outmap.find("output0");
                if (it0 != outmap.end()) pv = &it0->second;
                else                     pv = &outmap.begin()->second;

                const std::vector<float>& v = *pv;

                // 5) Decode: out0 = [1, C, N] with C = 4 + nc (no obj)
                auto sigmoid = [](float x){ return 1.f / (1.f + std::exp(-x)); };

                const int nc = 2;          // PEDRo export used nc=2
                const int C  = 4 + nc;     // 6
                if (v.size() % C != 0) {
                    ofLogError() << "[YOLO] unexpected output length=" << v.size() << " not divisible by C=" << C;
                    yolo_dets.clear(); return;
                }
                const int N = (int)(v.size() / C);
                // Flat memory is (C,N) contiguous for [1,C,N]
                auto at = [&](int c, int i)->float { return v[c * N + i]; };

                const bool coords_normalized = false;  // Ultralytics head → usually pixel coords in model space

                std::vector<Det> dets; dets.reserve(128);
                for (int i = 0; i < N; ++i) {
                    float cx = at(0,i), cy = at(1,i), w = at(2,i), h = at(3,i);
                    if (coords_normalized) { cx *= Wd; cy *= Hd; w *= Wd; h *= Hd; }

                    // best class prob (sigmoid per-class; not softmax)
                    int   best_cls = -1;
                    float best_p   = -1.f;
                    for (int c = 0; c < nc; ++c) {
                        float p = sigmoid(at(4 + c, i));
                        if (p > best_p) { best_p = p; best_cls = c; }
                    }
                    if (best_p < yolo_conf_thresh) continue;

                    // sanity filters in model space
                    if (w <= 1.f || h <= 1.f) continue;
                    float ar = w / std::max(1.f, h);
                    if (ar < 0.15f || ar > 6.7f) continue;

                    Det d;
                    d.x1 = cx - 0.5f*w;  d.y1 = cy - 0.5f*h;
                    d.x2 = cx + 0.5f*w;  d.y2 = cy + 0.5f*h;
                    d.score = best_p;
                    d.cls   = best_cls;
                    dets.push_back(d);
                }

                // 6) NMS
                auto kept = nms(std::move(dets), yolo_iou_thresh);

                // ---- define unletterboxAndProject BEFORE using it ----
                /*auto unletterboxAndProject = [&](float x1, float y1, float x2, float y2)->ofRectangle {
                    // invert letterbox: remove padding, then divide by scale
                    float lx1 = (x1 - lb_padx) / lb_scale;
                    float ly1 = (y1 - lb_pady) / lb_scale;
                    float lx2 = (x2 - lb_padx) / lb_scale;
                    float ly2 = (y2 - lb_pady) / lb_scale;

                    // clamp to sensor
                    auto clampf = [](float v, float a, float b){ return std::max(a, std::min(b, v)); };
                    lx1 = clampf(lx1, 0.f, (float)sensorW);
                    ly1 = clampf(ly1, 0.f, (float)sensorH);
                    lx2 = clampf(lx2, 0.f, (float)sensorW);
                    ly2 = clampf(ly2, 0.f, (float)sensorH);

                    // project sensor → screen (full-screen stretch)
                    float sx = lx1 * (drawW / sensorW);
                    float sy = ly1 * (drawH / sensorH);
                    float sw = (lx2 - lx1) * (drawW / sensorW);
                    float sh = (ly2 - ly1) * (drawH / sensorH);
                    return ofRectangle(sx, sy, sw, sh);
                };*/
                auto toSensor = [&](float x1, float y1, float x2, float y2)->ofRectangle {
                    float sx1 = (x1 - lb_padx) / lb_scale;
                    float sy1 = (y1 - lb_pady) / lb_scale;
                    float sx2 = (x2 - lb_padx) / lb_scale;
                    float sy2 = (y2 - lb_pady) / lb_scale;

                    sx1 = std::max(0.f, std::min((float)sensorW, sx1));
                    sy1 = std::max(0.f, std::min((float)sensorH, sy1));
                    sx2 = std::max(0.f, std::min((float)sensorW, sx2));
                    sy2 = std::max(0.f, std::min((float)sensorH, sy2));

                    return ofRectangle(sx1, sy1, sx2 - sx1, sy2 - sy1);
                };


                // 7) Un-letterbox to sensor, then project to screen and collect current dets
                /*std::vector<YoloDet> cur_screen;
                cur_screen.reserve(kept.size());
                for (auto &k : kept) {
                    auto r = unletterboxAndProject(k.x1, k.y1, k.x2, k.y2);
                    if (r.getWidth() > 0 && r.getHeight() > 0) {
                        cur_screen.push_back( YoloDet{ r, k.score, k.cls } );
                    }
                }*/
                // 7) Un-letterbox to sensor,
                std::vector<YoloDet> cur_sensor;
                cur_sensor.reserve(kept.size());
                for (auto &k : kept) {
                    auto r = toSensor(k.x1, k.y1, k.x2, k.y2);
                    if (r.getWidth() > 0 && r.getHeight() > 0) {
                        cur_sensor.push_back( YoloDet{ r, k.score, k.cls } );
                    }
                }

                // 8) Temporal smoothing over last 3 frames
                // NOTE: do NOT clear yolo_dets after this!
                yolo_dets = temporalSmooth3(cur_sensor, yolo_hist_,
                                            /*max_hist*/3,
                                            /*IoU*/0.5f, /*min_hits*/2,
                                            /*min_w*/12.f, /*min_h*/12.f);

                // Optional logging
                for (auto &d : yolo_dets) {
                    ofLogNotice() << "[YOLO] det cls=" << d.cls << " score=" << d.score
                                << " rect=" << d.box;
                }

                // newImageGen = false; // if you want one-shot per generated image

            } catch (const std::exception& e) {
                ofLogError() << "[YOLO] Inference error: " << e.what();
                yolo_dets.clear();
            }
        } else {
            yolo_dets.clear();
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
    }
}


void ofxDVS::onToggleEvent(ofxDatGuiToggleEvent e)
{
    if (e.target->is("ENABLE TRACKER")) {
        auto checked = e.target->getChecked();
        this->tracker_panel->setVisible(checked);
        this->enableTracker(checked);
        //hot pixels annoying
        this->rectangularClusterTrackerConfig.useVelocity = true;                 // already true
        this->rectangularClusterTrackerConfig.thresholdVelocityForVisibleCluster = 30.f; // px/s (tune 10..100)

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
        yolo_draw = e.target->getChecked();
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
    // onToggleEvent
    else if (e.target->getLabel() == "YOLO Conf") {
        yolo_conf_thresh = e.value;
    }
    else if (e.target->getLabel() == "VTEI Window (ms)") {
        vtei_win_ms = e.value;
        vtei_win_us = (long)std::round(vtei_win_ms * 1000.0f); // ms -> us
        ofLogNotice() << "VTEI window set to " << vtei_win_ms << " ms (" << vtei_win_us << " us)";
    }else if (e.target->getLabel() == "Refractory (us)") {
        hot_refrac_us = (int)e.value;
    }

    myCam.reset(); // no mesh turning when using GUI
}

// hot pixels
void ofxDVS::applyRefractory_() {
    const int W = sizeX, H = sizeY;
    for (auto &e : packetsPolarity) {
        if (!e.valid) continue;
        int x = (int)e.pos.x, y = (int)e.pos.y;
        if ((unsigned)x >= (unsigned)W || (unsigned)y >= (unsigned)H) { e.valid = false; continue; }
        int idx = y * W + x;
        int64_t last = last_ts_map_[idx];
        if (last != 0 && (e.timestamp - last) < hot_refrac_us) {
            e.valid = false;              // too soon → likely hot pixel burst
        } else {
            last_ts_map_[idx] = e.timestamp;
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
