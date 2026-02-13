#pragma once
/// @file dvs_gui.hpp
/// @brief GUI panel creation and event handlers for the NN and tracker panels.
///
/// These free functions create ofxDatGui panels and handle their events,
/// keeping the GUI wiring out of the main ofxDVS class.

#include "ofMain.h"
#include "ofxDatGui.h"

// Forward declarations
class ofxDVS;

namespace dvs { namespace gui {

/// Create the Neural Net panel (YOLO + TSDT folders) and attach event handlers.
/// Returns the panel; ownership is transferred to the caller.
std::unique_ptr<ofxDatGui> createNNPanel(ofxDVS* dvs);

/// Create the Rectangular Cluster Tracker panel and attach event handlers.
/// Returns the panel; ownership is transferred to the caller.
std::unique_ptr<ofxDatGui> createTrackerPanel(ofxDVS* dvs);

/// Create the Optical Flow panel and attach event handlers.
/// Returns the panel; ownership is transferred to the caller.
std::unique_ptr<ofxDatGui> createOptFlowPanel(ofxDVS* dvs);

// Event handler callbacks (bound to the panels)
void onNNToggleEvent(ofxDatGuiToggleEvent e, ofxDVS* dvs);
void onNNSliderEvent(ofxDatGuiSliderEvent e, ofxDVS* dvs);
void onNNButtonEvent(ofxDatGuiButtonEvent e, ofxDVS* dvs);

void onOptFlowToggleEvent(ofxDatGuiToggleEvent e, ofxDVS* dvs);
void onOptFlowSliderEvent(ofxDatGuiSliderEvent e, ofxDVS* dvs);

void onTrackerToggleEvent(ofxDatGuiToggleEvent e, ofxDVS* dvs);
void onTrackerSliderEvent(ofxDatGuiSliderEvent e, ofxDVS* dvs);

}} // namespace dvs::gui
