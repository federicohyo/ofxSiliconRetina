#include "dvs_gui.hpp"
#include "ofxDVS.hpp"

namespace dvs { namespace gui {

// ---- NN Panel ----
std::unique_ptr<ofxDatGui> createNNPanel(ofxDVS* dvs) {
    auto panel = std::make_unique<ofxDatGui>(ofxDatGuiAnchor::TOP_RIGHT);
    panel->setVisible(false);

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
    tsdt_folder->addButton("SELFTEST (from file)");

    // TP Detector folder
    auto tpdet_folder = panel->addFolder(">> TP Detector");
    tpdet_folder->addToggle("ENABLE TPDET", false);
    tpdet_folder->addToggle("DRAW TPDET", dvs->tpdet_pipeline.cfg.draw);
    tpdet_folder->addSlider("TPDET CONF", 0.0, 1.0, dvs->tpdet_pipeline.cfg.conf_thresh);
    tpdet_folder->addSlider("TPDET IOU",  0.0, 1.0, dvs->tpdet_pipeline.cfg.iou_thresh);
    tpdet_folder->addSlider("TPDET SMOOTH", 1, 5, dvs->tpdet_pipeline.cfg.smooth_frames);
    tpdet_folder->addButton("TPDET CLEAR HISTORY");

    panel->setPosition(270, 0);

    // Bind events using lambdas that capture `dvs`
    panel->onToggleEvent([dvs](ofxDatGuiToggleEvent e) { onNNToggleEvent(e, dvs); });
    panel->onSliderEvent([dvs](ofxDatGuiSliderEvent e) { onNNSliderEvent(e, dvs); });
    panel->onButtonEvent([dvs](ofxDatGuiButtonEvent e) { onNNButtonEvent(e, dvs); });

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
    } else if (name == "ENABLE TPDET") {
        dvs->tpdetEnabled = checked;
        if (!checked) dvs->tpdet_pipeline.clearHistory();
        ofLogNotice() << "TP Detector " << (checked ? "enabled" : "disabled");
    } else if (name == "DRAW TPDET") {
        dvs->tpdet_pipeline.cfg.draw = checked;
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
    } else if (n == "TPDET CONF") {
        dvs->tpdet_pipeline.cfg.conf_thresh = e.value;
    } else if (n == "TPDET IOU") {
        dvs->tpdet_pipeline.cfg.iou_thresh = e.value;
    } else if (n == "TPDET SMOOTH") {
        dvs->tpdet_pipeline.cfg.smooth_frames = std::max(1, (int)std::round(e.value));
    }
}

void onNNButtonEvent(ofxDatGuiButtonEvent e, ofxDVS* dvs) {
    if (e.target->getName() == "CLEAR HISTORY") {
        dvs->yolo_pipeline.clearHistory();
        ofLogNotice() << "YOLO temporal history cleared.";
    } else if (e.target->getName() == "SELFTEST (from file)") {
        dvs->tsdt_pipeline.debugFromFile(ofToDataPath("tsdt_input_fp32.bin", true));
    } else if (e.target->getName() == "TPDET CLEAR HISTORY") {
        dvs->tpdet_pipeline.clearHistory();
        ofLogNotice() << "TP Det temporal history cleared.";
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

    panel->setPosition(540, 0);

    panel->onToggleEvent([dvs](ofxDatGuiToggleEvent e) { onOptFlowToggleEvent(e, dvs); });
    panel->onSliderEvent([dvs](ofxDatGuiSliderEvent e) { onOptFlowSliderEvent(e, dvs); });

    return panel;
}

void onOptFlowToggleEvent(ofxDatGuiToggleEvent e, ofxDVS* dvs) {
    if (e.target->getName() == "DRAW FLOW") {
        dvs->drawOptFlow = e.target->getChecked();
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
