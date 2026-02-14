#include "dvs_gui.hpp"
#include "ofxDVS.hpp"

namespace dvs { namespace gui {

// ---- NN Panel ----
std::unique_ptr<ofxDatGui> createNNPanel(ofxDVS* dvs) {
    auto panel = std::make_unique<ofxDatGui>(ofxDatGuiAnchor::TOP_RIGHT);

    // Filters folder (always visible, independent of NN enable)
    auto filt = panel->addFolder(">> Filters");
    filt->addSlider("Refractory (us)", 0, 5000, dvs->hot_refrac_us);
    filt->addSlider("Hot Rate Window (ms)", 10, 1000, dvs->hot_rate_window_us / 1000);
    filt->addSlider("Hot Rate Threshold", 10, 5000, dvs->hot_rate_threshold);
    filt->addButton("Recalibrate Hot Pixels");
    filt->addSlider("BA Filter dt", 1, 100000, dvs->BAdeltaT);

    // YOLO folder
    auto nn_folder = panel->addFolder(">> Neural Net (YOLO)");
    nn_folder->addToggle("ENABLE NN", dvs->yolo_pipeline.cfg.draw);
    nn_folder->addToggle("DRAW DETECTIONS", dvs->yolo_pipeline.cfg.draw);
    nn_folder->addToggle("SHOW LABELS", dvs->yolo_pipeline.cfg.show_labels);
    nn_folder->addSlider("CONF THRESH", 0.0, 1.0, dvs->yolo_pipeline.cfg.conf_thresh);
    nn_folder->addSlider("IOU THRESH",  0.0, 1.0, dvs->yolo_pipeline.cfg.iou_thresh);
    nn_folder->addSlider("SMOOTH FRAMES", 1, 5, dvs->yolo_pipeline.cfg.smooth_frames);
    nn_folder->addSlider("VTEI Window (ms)", 5, 200, dvs->yolo_pipeline.cfg.vtei_win_ms);
    nn_folder->addButton("CLEAR HISTORY");

    // TSDT folder
    auto tsdt_folder = panel->addFolder(">> Neural Net (TSDT)");
    tsdt_folder->addToggle("ENABLE TSDT", dvs->tsdtEnabled);
    tsdt_folder->addToggle("SHOW LABEL",  dvs->tsdt_pipeline.cfg.show_label);
    tsdt_folder->addSlider("TIMESTEPS (T)", 1, 16, dvs->tsdt_pipeline.cfg.T);
    tsdt_folder->addSlider("BIN (ms)",      1, 50,  dvs->tsdt_pipeline.cfg.bin_ms);
    tsdt_folder->addSlider("EMA alpha",     0.0, 1.0, dvs->tsdt_pipeline.cfg.ema_alpha);
    tsdt_folder->addSlider("Confidence %", 0, 100, dvs->tsdt_pipeline.cfg.conf_threshold * 100.f);
    tsdt_folder->addSlider("Display (s)",  0.1, 5.0, dvs->tsdt_pipeline.cfg.display_timeout);
    tsdt_folder->addButton("SELFTEST (from file)");

    // TPDVSGesture folder
    auto tpg_folder = panel->addFolder(">> TPDVSGesture");
    tpg_folder->addToggle("ENABLE TPDVSGesture", dvs->tpdvsGestureEnabled);
    tpg_folder->addToggle("TPDVSGesture SHOW LABEL", dvs->tpdvs_gesture_pipeline.cfg.show_label);
    tpg_folder->addSlider("TPDVSGesture EMA", 0.0, 1.0, dvs->tpdvs_gesture_pipeline.cfg.ema_alpha);
    tpg_folder->addSlider("TPDVSGesture Window (ms)", 10, 200, dvs->tpdvs_gesture_pipeline.cfg.bin_window_ms);
    tpg_folder->addSlider("TPDVSGesture Confidence %", 0, 100, dvs->tpdvs_gesture_pipeline.cfg.conf_threshold * 100.f);
    tpg_folder->addSlider("TPDVSGesture Display (s)", 0.1, 5.0, dvs->tpdvs_gesture_pipeline.cfg.display_timeout);
    tpg_folder->addButton("TPDVSGesture CLEAR HISTORY");

    panel->setPosition(270, 0);

    // Bind events using lambdas that capture `dvs`
    panel->onToggleEvent([dvs](ofxDatGuiToggleEvent e) { onNNToggleEvent(e, dvs); });
    panel->onSliderEvent([dvs](ofxDatGuiSliderEvent e) {
        onFilterSliderEvent(e, dvs);
        onNNSliderEvent(e, dvs);
    });
    panel->onButtonEvent([dvs](ofxDatGuiButtonEvent e) {
        onFilterButtonEvent(e, dvs);
        onNNButtonEvent(e, dvs);
    });

    return panel;
}

// ---- Tracker Panel ----
std::unique_ptr<ofxDatGui> createTrackerPanel(ofxDVS* dvs) {
    auto panel = std::make_unique<ofxDatGui>(ofxDatGuiAnchor::TOP_RIGHT);
    panel->setVisible(false);

    auto& cfg = dvs->rectangularClusterTrackerConfig;
    auto f = panel->addFolder(">> Tracker Controls");

    f->addToggle("FILTER", cfg.filterEnabled);
    f->addSlider("UPDATE INTERVAL ms", 0, 1000, cfg.updateIntervalMs);
    f->addSlider("MAX NUM CLUSTERS", 1, 100, cfg.maxNumClusters);
    f->addBreak()->setHeight(5.0f);
    f->addToggle("ELLIPTICAL CLUSTERS", cfg.useEllipticalClusters);
    f->addSlider("PATH LENGTH", 1, 500, cfg.pathLength);
    f->addToggle("SHOW CLUSTER NUMBER", cfg.showClusterNumber);
    f->addToggle("SHOW CLUSTER EPS", cfg.showClusterEps);
    f->addToggle("SHOW CLUSTER RADIUS", cfg.showClusterRadius);
    f->addToggle("SHOW CLUSTER VELOCITY", cfg.showClusterVelocity);
    f->addToggle("SHOW CLUSTER VEL VECTOR", cfg.showClusterVelocityVector);
    f->addToggle("SHOW CLUSTER MASS", cfg.showClusterMass);
    f->addToggle("SHOW PATHS", cfg.showPaths);
    f->addSlider("VELOCITY VECTOR SCALING", 0, 10, cfg.velocityVectorScaling);
    f->addBreak()->setHeight(5.0f);
    f->addSlider("MIXING FACTOR", 0, 1, cfg.mixingFactor);
    f->addToggle("PATHS", cfg.pathsEnabled);
    f->addToggle("USE VELOCITY", cfg.useVelocity);
    f->addToggle("USE NEAREST CLUSTER", cfg.useNearestCluster);
    f->addSlider("PREDICTIVE VELOCITY", 0, 100, cfg.predictiveVelocityFactor);
    f->addToggle("initializeVelocityToAverage", cfg.initializeVelocityToAverage);
    f->addSlider("VELOCITY TAU ms", 0, 1000, cfg.velocityTauMs);
    f->addSlider("FRICTION TAU ms", 0, 1000, cfg.frictionTauMs);
    f->addBreak()->setHeight(5.0f);
    f->addSlider("SURROUND", 0, 10, cfg.surround);
    f->addToggle("DYNAMIC SIZE", cfg.dynamicSizeEnabled);
    f->addToggle("DYNAMIC ASPECT RATIO", cfg.dynamicAspectRatioEnabled);
    f->addToggle("DYNAMIC ANGLE", cfg.dynamicAngleEnabled);
    f->addSlider("ASPECT RATIO", 0, 2, cfg.aspectRatio);
    f->addSlider("CLUSTER SIZE", 0, 2, cfg.clusterSize);
    f->addToggle("HIGHWAY PERSPECTIVE", cfg.highwayPerspectiveEnabled);
    f->addToggle("ANGLE FOLLOWS VELOCITY", cfg.angleFollowsVelocity);
    f->addBreak()->setHeight(5.0f);
    f->addToggle("ONE POLARITY", cfg.useOnePolarityOnlyEnabled);
    f->addToggle("GROW MERGED SIZE", cfg.growMergedSizeEnabled);
    f->addSlider("velAngDiffDegToNotMerge", 0, 360, cfg.velAngDiffDegToNotMerge);
    f->addBreak()->setHeight(5.0f);
    f->addSlider("THRESHOLD MASS", 0, 100, cfg.thresholdMassForVisibleCluster);
    f->addSlider("THRESHOLD VELOCITY", 0, 100, cfg.thresholdVelocityForVisibleCluster);
    f->addSlider("MASS DECAY TAU us", 0, 100000, cfg.clusterMassDecayTauUs);
    f->addToggle("CLUSTER EXIT PURGING", cfg.enableClusterExitPurging);
    f->addToggle("SURROUND INHIBITION", cfg.surroundInhibitionEnabled);
    f->addSlider("SURROUND INHIBITION COST", 0, 10, cfg.surroundInhibitionCost);
    f->addToggle("DO NOT MERGE", cfg.dontMergeEver);
    f->addBreak()->setHeight(5.0f);
    f->addToggle("SMOOTH MOVE", cfg.smoothMove);
    f->addSlider("SMOOTH WEIGHT", 0, 1000, cfg.smoothWeight);
    f->addSlider("SMOOTH POSITION", 0, 0.1f, cfg.smoothPosition);
    f->addSlider("SMOOTH INTEGRAL", 0, 0.1f, cfg.smoothIntegral);

    panel->onToggleEvent([dvs](ofxDatGuiToggleEvent e) { onTrackerToggleEvent(e, dvs); });
    panel->onSliderEvent([dvs](ofxDatGuiSliderEvent e) { onTrackerSliderEvent(e, dvs); });

    return panel;
}

// ---- Filter event handlers ----
void onFilterSliderEvent(ofxDatGuiSliderEvent e, ofxDVS* dvs) {
    const std::string& n = e.target->getName();
    if (n == "Refractory (us)") {
        dvs->hot_refrac_us = (int)e.value;
    } else if (n == "Hot Rate Window (ms)") {
        dvs->hot_rate_window_us = (int)e.value * 1000;
        ofLogNotice() << "[HotPixel] Rate window set to " << dvs->hot_rate_window_us << " us";
    } else if (n == "Hot Rate Threshold") {
        dvs->hot_rate_threshold = (int)e.value;
        ofLogNotice() << "[HotPixel] Rate threshold set to " << dvs->hot_rate_threshold;
    } else if (n == "BA Filter dt") {
        dvs->changeBAdeltat(e.value);
    }
}

void onFilterButtonEvent(ofxDatGuiButtonEvent e, ofxDVS* dvs) {
    if (e.target->getName() == "Recalibrate Hot Pixels") {
        dvs->recalibrateHotPixels();
    }
}

// ---- NN event handlers ----
void onNNToggleEvent(ofxDatGuiToggleEvent e, ofxDVS* dvs) {
    const std::string& name = e.target->getName();
    bool checked = e.target->getChecked();

    if (name == "ENABLE NN") {
        dvs->nnEnabled = checked;
        ofLogNotice() << "NN execution " << (checked ? "enabled" : "disabled");
    } else if (name == "DRAW DETECTIONS") {
        dvs->yolo_pipeline.cfg.draw = checked;
    } else if (name == "SHOW LABELS") {
        dvs->yolo_pipeline.cfg.show_labels = checked;
    } else if (name == "ENABLE TSDT") {
        dvs->tsdtEnabled = checked;
        if (!checked) dvs->tsdt_pipeline.clearHistory();
        ofLogNotice() << "TSDT execution " << (checked ? "enabled" : "disabled");
    } else if (name == "SHOW LABEL") {
        dvs->tsdt_pipeline.cfg.show_label = checked;
    } else if (name == "ENABLE TPDVSGesture") {
        dvs->tpdvsGestureEnabled = checked;
        if (!checked) dvs->tpdvs_gesture_pipeline.clearHistory();
        ofLogNotice() << "TPDVSGesture " << (checked ? "enabled" : "disabled");
    } else if (name == "TPDVSGesture SHOW LABEL") {
        dvs->tpdvs_gesture_pipeline.cfg.show_label = checked;
    }
}

void onNNSliderEvent(ofxDatGuiSliderEvent e, ofxDVS* dvs) {
    const std::string& n = e.target->getName();

    if (n == "CONF THRESH") {
        dvs->yolo_pipeline.cfg.conf_thresh = e.value;
    } else if (n == "IOU THRESH") {
        dvs->yolo_pipeline.cfg.iou_thresh = e.value;
    } else if (n == "SMOOTH FRAMES") {
        dvs->yolo_pipeline.cfg.smooth_frames = std::max(1, (int)std::round(e.value));
    } else if (n == "VTEI Window (ms)") {
        dvs->yolo_pipeline.cfg.vtei_win_ms = e.value;
        ofLogNotice() << "VTEI window: " << e.value << " ms";
    } else if (n == "TIMESTEPS (T)") {
        dvs->tsdt_pipeline.cfg.T = std::max(1, (int)std::round(e.value));
    } else if (n == "BIN (ms)") {
        dvs->tsdt_pipeline.cfg.bin_ms = std::max(1, (int)std::round(e.value));
    } else if (n == "EMA alpha") {
        dvs->tsdt_pipeline.cfg.ema_alpha = ofClamp((float)e.value, 0.f, 1.f);
    } else if (n == "Confidence %") {
        dvs->tsdt_pipeline.cfg.conf_threshold = (float)e.value / 100.f;
    } else if (n == "Display (s)") {
        dvs->tsdt_pipeline.cfg.display_timeout = (float)e.value;
    } else if (n == "TPDVSGesture EMA") {
        dvs->tpdvs_gesture_pipeline.cfg.ema_alpha = ofClamp((float)e.value, 0.f, 1.f);
    } else if (n == "TPDVSGesture Window (ms)") {
        dvs->tpdvs_gesture_pipeline.cfg.bin_window_ms = (float)e.value;
    } else if (n == "TPDVSGesture Confidence %") {
        dvs->tpdvs_gesture_pipeline.cfg.conf_threshold = (float)e.value / 100.f;
    } else if (n == "TPDVSGesture Display (s)") {
        dvs->tpdvs_gesture_pipeline.cfg.display_timeout = (float)e.value;
    }
}

void onNNButtonEvent(ofxDatGuiButtonEvent e, ofxDVS* dvs) {
    if (e.target->getName() == "CLEAR HISTORY") {
        dvs->yolo_pipeline.clearHistory();
        ofLogNotice() << "YOLO temporal history cleared.";
    } else if (e.target->getName() == "SELFTEST (from file)") {
        dvs->tsdt_pipeline.debugFromFile(ofToDataPath("tsdt_input_fp32.bin", true));
    } else if (e.target->getName() == "TPDVSGesture CLEAR HISTORY") {
        dvs->tpdvs_gesture_pipeline.clearHistory();
        ofLogNotice() << "TPDVSGesture history cleared.";
    }
}

// ---- Optical Flow Panel ----
std::unique_ptr<ofxDatGui> createOptFlowPanel(ofxDVS* dvs) {
    auto panel = std::make_unique<ofxDatGui>(ofxDatGuiAnchor::TOP_RIGHT);
    panel->setVisible(false);

    auto f = panel->addFolder(">> Optical Flow");
    f->addToggle("DRAW FLOW", dvs->drawOptFlow);
    f->addSlider("FLOW DECAY",     0.80, 1.0,  dvs->optFlowDecay);
    f->addSlider("FLOW RADIUS",    1,    5,    dvs->optFlowRadius);
    f->addSlider("FLOW DT (ms)",   5,    200,  dvs->optFlowDt_us / 1000);
    f->addSlider("FLOW MAX SPEED", 10,   2000, dvs->optFlowMaxSpeed);

    auto r = panel->addFolder(">> Reconstruction");
    r->addToggle("RECON IMAGE", dvs->drawRecon);
    r->addSlider("RECON DECAY",   0.90, 1.0,  dvs->reconDecay);
    r->addSlider("RECON CONTRIB", 0.01, 0.5,  dvs->reconContrib);
    r->addSlider("RECON SPREAD",  1,    8,    dvs->reconSpread);

    panel->setPosition(540, 0);

    panel->onToggleEvent([dvs](ofxDatGuiToggleEvent e) { onOptFlowToggleEvent(e, dvs); });
    panel->onSliderEvent([dvs](ofxDatGuiSliderEvent e) { onOptFlowSliderEvent(e, dvs); });

    return panel;
}

void onOptFlowToggleEvent(ofxDatGuiToggleEvent e, ofxDVS* dvs) {
    if (e.target->getName() == "DRAW FLOW") {
        dvs->drawOptFlow = e.target->getChecked();
    } else if (e.target->getName() == "RECON IMAGE") {
        dvs->drawRecon = e.target->getChecked();
    }
}

void onOptFlowSliderEvent(ofxDatGuiSliderEvent e, ofxDVS* dvs) {
    const std::string& n = e.target->getName();
    if (n == "FLOW DECAY") {
        dvs->optFlowDecay = e.value;
    } else if (n == "FLOW RADIUS") {
        dvs->optFlowRadius = (int)e.value;
    } else if (n == "FLOW DT (ms)") {
        dvs->optFlowDt_us = (int)e.value * 1000;
    } else if (n == "FLOW MAX SPEED") {
        dvs->optFlowMaxSpeed = e.value;
    } else if (n == "RECON DECAY") {
        dvs->reconDecay = e.value;
    } else if (n == "RECON CONTRIB") {
        dvs->reconContrib = e.value;
    } else if (n == "RECON SPREAD") {
        dvs->reconSpread = (int)e.value;
    }
}

// ---- Tracker event handlers ----
void onTrackerSliderEvent(ofxDatGuiSliderEvent e, ofxDVS* dvs) {
    auto& cfg = dvs->rectangularClusterTrackerConfig;
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
    if (it != vars.end()) *(it->second) = e.target->getValue();
}

void onTrackerToggleEvent(ofxDatGuiToggleEvent e, ofxDVS* dvs) {
    auto& cfg = dvs->rectangularClusterTrackerConfig;
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
        { "CLUSTER EXIT PURGING",   &cfg.enableClusterExitPurging },
        { "SURROUND INHIBITION",    &cfg.surroundInhibitionEnabled },
        { "DO NOT MERGE",           &cfg.dontMergeEver },
        { "SMOOTH MOVE",            &cfg.smoothMove },
    };
    auto it = vars.find(e.target->getName());
    if (it != vars.end()) *(it->second) = e.target->getChecked();
}

}} // namespace dvs::gui
