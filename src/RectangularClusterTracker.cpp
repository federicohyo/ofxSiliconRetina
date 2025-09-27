#include "RectangularClusterTracker.hpp"

int RectangularClusterTracker::Cluster::clusterCounter = 0;
float RectangularClusterTracker::Cluster::initialAngle = 0;

// ---------------------------------------------------------------------------
void RectangularClusterTracker::LowpassFilter::setTauMs(float tau) {
    if(tau<0) tau=0;
    tauMs = tau;
}
// ---------------------------------------------------------------------------
float RectangularClusterTracker::LowpassFilter::filter(float val, int64_t time) {
    if (!initialized) {
        lpVal = val;
        lastVal = val;
        lastTime = time;
        initialized = true;
        return val;
    }
    if (tauMs == 0) {
        lpVal = val;
        lastVal = val;
        return lpVal;
    }
    int64_t dt = time - lastTime;
    if (dt < 0) {
        dt = 0;
    }
    lastTime = time;
    float fac = (float) dt / tauMs / TICK_PER_MS;
    if (fac > 1) {
        fac = 1;
    }
    lpVal = lpVal + (val - lpVal) * fac;
    lastVal = val;
    return lpVal;
}
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
RectangularClusterTracker::Cluster::Cluster(RectangularClusterTracker& tracker) : tracker(tracker), cfg(tracker.cfg) {
    float hue = ofRandomuf();
    color.setHsb(hue, 1.0f, 1.0f);
    clusterNumber = ++clusterCounter;
    aspectRatio = cfg.aspectRatio;
    vxFilter.setTauMs(cfg.velocityTauMs);
    vyFilter.setTauMs(cfg.velocityTauMs);
    if (cfg.initializeVelocityToAverage) {
        velocityPPT.x = tracker.averageVelocityPPT.x;
        velocityPPT.y = tracker.averageVelocityPPT.y;
        velocityValid = true;
    }
    setAngle(initialAngle);
    setRadius(tracker.defaultClusterRadius);
}
// ---------------------------------------------------------------------------
RectangularClusterTracker::Cluster::Cluster(RectangularClusterTracker& tracker, const PolarityEvent& ev) : Cluster(tracker) {
    location.x = ev.x;
    location.y = ev.y;
    birthLocation.x = ev.x;
    birthLocation.y = ev.y;
    lastPacketLocation.x = ev.x;
    lastPacketLocation.y = ev.y;
    lastEventTimestamp = ev.timestamp;
    lastUpdateTime = ev.timestamp;
    firstEventTimestamp = lastEventTimestamp;
    numEvents = 1;
    mass = 1;
    setRadius(tracker.defaultClusterRadius);
}
// ---------------------------------------------------------------------------
RectangularClusterTracker::Cluster::Cluster(RectangularClusterTracker& tracker, const PolarityEvent& ev, PolaritiesQueue& output)
    : Cluster(tracker, ev) {
    if (!isVisible()) {
        return;
    }
    output.push_back(ev);
}
// ---------------------------------------------------------------------------
/**
 * Constructs a cluster by merging two clusters. All parameters of the
 * resulting cluster should be reasonable combinations of the source
 * cluster parameters. For example, the merged location values are
 * weighted by the mass of events that have supported each source
 * cluster, so that older clusters weigh more heavily in the resulting
 * cluster location. Subtle bugs or poor performance can result from not
 * properly handling the merging of parameters.
 *
 * @param one the first cluster
 * @param two the second cluster
 */
RectangularClusterTracker::Cluster::Cluster(RectangularClusterTracker& tracker, const Cluster& one, const Cluster& two) : Cluster(tracker) {
    mergeTwoClustersToThis(one, two);
}
// ---------------------------------------------------------------------------
void RectangularClusterTracker::Cluster::addEvent(const PolarityEvent& ev, PolaritiesQueue& output) {
    addEvent(ev);
    if (!isVisible()) {
        return;
    }
    PolarityEvent oe = ev;
    // oe.setCluster(this);
    output.push_back(oe);
}
// ---------------------------------------------------------------------------
void RectangularClusterTracker::Cluster::addEvent(const PolarityEvent& ev) {
    if (cfg.useOnePolarityOnlyEnabled) {
        if (ev.polarity == true) {
            return;
        } 
    } else {
        if (ev.polarity == false) {
            return;
        }
    }

    updateMass(ev.timestamp);
    const float m = cfg.mixingFactor;
    updatePosition(ev, m);
    updateEventRate(ev, m);
    updateAverageEventDistance(m);
    updateShape(ev);
    lastUpdateTime = ev.timestamp;
}
// ---------------------------------------------------------------------------
void RectangularClusterTracker::Cluster::drawEllipse(float centerX, float centerY, float radiusX, float radiusY, float angle, int N) {
    ofPushMatrix();
    ofTranslate(centerX, centerY, 0);
    if (angle != 0) {
        ofRotateRad(angle, 0, 0, 1);
    }

    glBegin(GL_LINE_LOOP);
    {
        for (int i = 0; i < N; i++) {
            double a = ((float) i / N) * 2 * M_PI;
            double cosA = std::cos(a);
            double sinA = std::sin(a);

            glVertex2d(radiusX * cosA, radiusY * sinA);
        }
    }
    glEnd();
    ofPopMatrix();
}
// ---------------------------------------------------------------------------
void RectangularClusterTracker::Cluster::drawCircle(float centerX, float centerY, float radius, int N) {
    drawEllipse(centerX, centerY, radius, radius, 0, N);
}
// ---------------------------------------------------------------------------
void RectangularClusterTracker::Cluster::drawBox(float centerX, float centerY, float width, float height, float angle) {
    ofPushMatrix();
    ofTranslate(centerX, centerY, 0);
    if (angle != 0) {
        ofRotateRad(angle, 0, 0, 1);
    }
    ofDrawRectangle(-width/2, -height/2, width, height);
    ofPopMatrix();
}
// ---------------------------------------------------------------------------
void RectangularClusterTracker::Cluster::drawLine(float startX, float startY, float lengthX, float lengthY, float scale) {
    ofPushMatrix();
    ofTranslate(startX, startY, 0);

    glBegin(GL_LINES);
    glVertex2f(0, 0);
    glVertex2f(lengthX * scale, lengthY * scale);
    glEnd();
    ofPopMatrix();
}
// ---------------------------------------------------------------------------
void RectangularClusterTracker::Cluster::drawVector(float origX, float origY, float headX, float headY, float headlength, float scale) {
    float endx = headX * scale, endy = headY * scale;
    float arx = -endx + endy, ary = -endx - endy;   // halfway between pointing back to origin
    float l = (float) std::sqrt((arx * arx) + (ary * ary)); // length
    arx = (arx / l) * headlength;
    ary = (ary / l) * headlength; // normalize to headlength

    ofPushMatrix();
    ofTranslate(origX, origY, 0);

    glBegin(GL_LINES);
    {
        glVertex2f(0, 0);
        glVertex2f(endx, endy);
        // draw arrow (half)
        glVertex2f(endx, endy);
        glVertex2f(endx + arx, endy + ary);
        // other half, 90 degrees
        glVertex2f(endx, endy);
        glVertex2f(endx + ary, endy - arx);
    }
    glEnd();
    ofPopMatrix();
}

static float drawBitmapString(const std::string& text, float x, float y) {
    static ofBitmapFont f;
    ofDrawBitmapString(text, x, y);
    ofRectangle r = f.getBoundingBox(text, x, y);
    return r.width;
}

// ---------------------------------------------------------------------------
void RectangularClusterTracker::Cluster::draw() {
    const int OUTLINE_PX = 3;   // visual thickness via multi-stroke
    const int ELL_N = 64;       // ellipse smoothness

    ofPushMatrix();
    ofTranslate(location.x, location.y);

    ofPushStyle();
    ofNoFill();
    ofSetColor(255, 215, 0);    // always yellow (gold)

    if (cfg.useEllipticalClusters) {
        // rotated ellipse: draw multiple loops to simulate thick stroke
        for (int k = 0; k < OUTLINE_PX; ++k) {
            drawEllipse(0, 0, radiusX + k, radiusY + k, angle, ELL_N);
        }
    } else {
        // rotated rectangle: draw multiple rectangles to simulate thick stroke
        ofPushMatrix();
        if (angle != 0) ofRotateRad(angle);
        for (int k = 0; k < OUTLINE_PX; ++k) {
            float w = (radiusX + k) * 2.0f;
            float h = (radiusY + k) * 2.0f;
            ofDrawRectangle(-w * 0.5f, -h * 0.5f, w, h);
        }
        ofPopMatrix();
    }

    // Optional axis line (same yellow)
    if ((angle != 0) || cfg.dynamicAngleEnabled) {
        drawLine(0, 0, radiusX, 0, 1);
    }

    // Optional: velocity vector in yellow too
    if (cfg.showClusterVelocityVector) {
        drawVector(0, 0, velocityPPS.x, velocityPPS.y, 3, cfg.velocityVectorScaling);
    }

    // Optional: radius circle (draw multi to look thick)
    if (cfg.showClusterRadius) {
        for (int k = 0; k < OUTLINE_PX; ++k) {
            drawCircle(0, 0, averageEventDistance + k, 64);
        }
    }

    ofPopStyle();
    ofPopMatrix();

    if (cfg.showPaths) {
        ofPushStyle();
        ofSetColor(255, 215, 0);
        glPointSize(3);
        glBegin(GL_POINTS);
        for (auto &p : path) glVertex2f(p.x, p.y);
        glEnd();
        ofPopStyle();
    }

    // text annotations on clusters, setup
    if (cfg.showClusterMass || cfg.showClusterEps || cfg.showClusterNumber || cfg.showClusterVelocity) {
        ofSetColor(isWasEverVisible() ? ofColor(255) : ofColor(140));

        if (isWasEverVisible()) {
            ofSetColor(255,255,255);
        } else {
            ofSetColor(140,140,140);
        }
        float x = location.x;
        float y = location.y;
        char str[64];

        if (cfg.showClusterVelocity) {
            snprintf(str, 64, "v(vx,vy)=%.0f(%.0f,%.0f) pps ", getSpeedPPS(), velocityPPS.x, velocityPPS.y);
            x += drawBitmapString(str, x, y);
        }
        if (cfg.showClusterEps) {
            snprintf(str, 64, "eps=%.0fk ", (avgEventRate / TICK_DEFAULT_US) * 1e3f);
            // annotate the cluster with the event rate computed as 1/(avg ISI) in keps
            x += drawBitmapString(str, x, y);
        }
        if (cfg.showClusterNumber) {
            snprintf(str, 64, "#=%d ", clusterNumber);
            x += drawBitmapString(str, x, y);
        }
        if (cfg.showClusterMass) {
            snprintf(str, 64, "m=%.1f ", getMassNow(lastUpdateTime));
            x += drawBitmapString(str, x, y);
        }
    }
}
// ---------------------------------------------------------------------------
void RectangularClusterTracker::Cluster::updateMass(int64_t t) {
    if (cfg.surroundInhibitionEnabled) {
        // if the event is in the surround, we decrement the mass, if inside cluster, we increment
        float normDistance = distanceToLastEvent / radius;
        // float dmass = normDistance <= 1 ? 1 : -1;
        float dmass = normDistance <= 1 ? 1 : -cfg.surroundInhibitionCost;
        mass = dmass + (mass * (float) std::exp((double) (lastEventTimestamp - t) / cfg.clusterMassDecayTauUs));
    } else {
        bool wasInfinite = std::isinf(mass);
        // don't worry about distance, just increment
        int64_t dt = lastEventTimestamp - t;
        if (dt < 0) {
            mass = 1 + (mass * (float) std::exp((double) dt / cfg.clusterMassDecayTauUs));
            if (!wasInfinite && std::isinf(mass)) {
                ofLog(OF_LOG_WARNING, "mass became infinite for cluster %d", clusterNumber);
            }
        }
    }
}
// ---------------------------------------------------------------------------
/**
 * Updates path (historical) information for this cluster, including
 * cluster velocity (by calling updateVelocity()). The path is trimmed
 * to maximum length if logging is not enabled.
 *
 * @param t current timestamp.
 */
void RectangularClusterTracker::Cluster::updatePath(int64_t t) {
    if (!cfg.pathsEnabled && !cfg.useVelocity) {
        return;
    }
    if (numEvents == previousNumEvents) {
        return; // don't appendCopy point unless we had events that caused change in path (aside from prediction from velocityPPT)
    }
    ClusterPathPoint p(location.x, location.y, t);
    p.nEvents = numEvents - previousNumEvents;
    path.push_back(p);
    previousNumEvents = numEvents;
    updateVelocity();

    if (path.size() > cfg.pathLength) {
        //if (!logDataEnabled || (clusterLoggingMethod != ClusterLoggingMethod.LogClusters)) {
            path.pop_front(); // if we're logging cluster paths, then save all cluster history regardless of pathLength
        //}
    }
}
// ---------------------------------------------------------------------------
/**
 * Updates velocityPPT, velocityPPS of cluster and last path point
 * lowpass filtered velocity.
 */
void RectangularClusterTracker::Cluster::updateVelocity() {
    if (path.size() < 2) {
        return;
    }

    // update velocityPPT of cluster using last two path points
    auto itr = path.rbegin();
    ClusterPathPoint plast = *itr++;
    int nevents = plast.nEvents;
    ClusterPathPoint pfirst = *itr++;
    while ((nevents < cfg.thresholdMassForVisibleCluster) && itr != path.rend()) {
        nevents += pfirst.nEvents;
        pfirst = *itr++;
    }
    if (nevents < cfg.thresholdMassForVisibleCluster) {
        return;
    }

    int64_t dt = plast.t - pfirst.t;
    float vx = (plast.x - pfirst.x) / dt;
    float vy = (plast.y - pfirst.y) / dt;
    velocityPPT.x = vxFilter.filter(vx, lastEventTimestamp);
    velocityPPT.y = vyFilter.filter(vy, lastEventTimestamp);
    if (!std::isnan(cfg.frictionTauMs) && cfg.frictionTauMs != 0) {
        float factor = std::exp(-dt / (cfg.frictionTauMs * 1000));
        velocityPPT.x = velocityPPT.x * factor;
        velocityPPT.y = velocityPPT.y * factor;
    }
    plast.velocityPPT = Point2D<float>(velocityPPT.x, velocityPPT.y);
    velocityPPS.x = velocityPPT.x * VELPPS_SCALING;
    velocityPPS.y = velocityPPT.y * VELPPS_SCALING;
    velocityValid = true;
}
// ---------------------------------------------------------------------------
void RectangularClusterTracker::Cluster::updateLocation(int64_t t, Point2D<float>& averageVelocityPPT) {
    if (isVelocityValid()) {
        int64_t dt = t - lastUpdateTime;
        if (dt <= 0) {
            return; // bogus timestamp or doesn't need update
        }
        location.x += velocityPPT.x * dt * cfg.predictiveVelocityFactor;
        location.y += velocityPPT.y * dt * cfg.predictiveVelocityFactor;
        if (cfg.initializeVelocityToAverage) {
            // update average velocity metric for construction of new Clusters
            averageVelocityPPT.x = ((1 - AVERAGE_VELOCITY_MIXING_FACTOR) * averageVelocityPPT.x)
                    + (AVERAGE_VELOCITY_MIXING_FACTOR * velocityPPT.x);
            averageVelocityPPT.y = ((1 - AVERAGE_VELOCITY_MIXING_FACTOR) * averageVelocityPPT.y)
                    + (AVERAGE_VELOCITY_MIXING_FACTOR * velocityPPT.y);
        }
        lastUpdateTime = t;
    }
}
// ---------------------------------------------------------------------------
/**
 * Returns true if the cluster center is outside the array or if this
 * test is enabled and if the cluster has hit the edge of the array and
 * has been there at least the minimum time for support.
 *
 * @return true if cluster has hit edge for long enough
 * (getClusterMassDecayTauUs) and test enableClusterExitPurging
 */
bool RectangularClusterTracker::Cluster::hasHitEdge() {
    if (!cfg.enableClusterExitPurging) {
        return false;
    }

    int lx = (int) location.x, ly = (int) location.y;
    int sx = tracker.chipSizeX, sy = tracker.chipSizeY;

    return ((lx <= 0) || (lx >= sx) || (ly <= 0) || (ly >= sy)); // always onPruning if cluster center is outside
    // array, e.g. from velocityPPT prediction
}
// ---------------------------------------------------------------------------
/**
 * Measures distance from cluster center to event.
 *
 * @param event
 * @return distance of this cluster to the event in Manhattan (cheap)
 * metric (sum of absolute values of x and y distance).
 */
float RectangularClusterTracker::Cluster::distanceTo(const PolarityEvent& event) const {
    const float dx = event.x - location.x;
    const float dy = event.y - location.y;

    return distanceMetric(dx, dy);
}
// ---------------------------------------------------------------------------
/**
 * Measures distance in x direction, accounting for instantaneousAngle
 * of cluster and predicted movement of cluster.
 *
 * @param event
 * @return distance in x direction of this cluster to the event, where x
 * is measured along instantaneousAngle=0.
 */
float RectangularClusterTracker::Cluster::distanceToX(const PolarityEvent& event) const {
    int64_t dt = event.timestamp - lastUpdateTime;
    float distance = std::abs((((event.x - location.x) + (velocityPPT.x * (dt))) * cosAngle)
            + (((event.y - location.y) + (velocityPPT.y * (dt))) * sinAngle));
    return distance;
}
// ---------------------------------------------------------------------------
/**
 * Measures distance in y direction, accounting for instantaneousAngle
 * of cluster, where y is measured along instantaneousAngle=Pi/2 and
 * predicted movement of cluster
 *
 * @param event
 * @return distance in y direction of this cluster to the event
 */
float RectangularClusterTracker::Cluster::distanceToY(const PolarityEvent& event) const {
    int64_t dt = event.timestamp - lastUpdateTime;
    float distance = std::abs((((event.y - location.y) + (velocityPPT.y * (dt))) * cosAngle)
            - (((event.x - location.x) + (velocityPPT.x * (dt))) * sinAngle));
    return distance;
}
// ---------------------------------------------------------------------------
/**
 * Computes and returns distance to another cluster.
 *
 * @param c
 * @return distance of this cluster to the other cluster in pixels.
 */
float RectangularClusterTracker::Cluster::distanceTo(const Cluster& c)const  {
    float dx = c.location.x - location.x;
    float dy = c.location.y - location.y;
    return distanceMetric(dx, dy);
}
// ---------------------------------------------------------------------------
/**
 * Computes and returns the angle of this cluster's velocityPPT vector
 * to another cluster's velocityPPT vector.
 *
 * @param c the other cluster.
 * @return the angle in radians, from 0 to PI in radians. If either
 * cluster has zero velocityPPT, returns 0.
 */
float RectangularClusterTracker::Cluster::velocityAngleToRad(const Cluster& c) const {
    float s1 = getSpeedPPS(), s2 = c.getSpeedPPS();
    if ((s1 == 0) || (s2 == 0)) {
        return 0;
    }
    float dot = (velocityPPS.x * c.velocityPPS.x) + (velocityPPS.y * c.velocityPPS.y);
    float angleRad = (float) std::acos(dot / s1 / s2);
    return angleRad;
}
// ---------------------------------------------------------------------------
/**
 * Computes and returns {@link #mass} at time t, using the last time an
 * event hit this cluster and the {@link #clusterMassDecayTauUs}. Does
 * not change the mass.
 *
 * @param t timestamp now.
 * @return the mass.
 */
float RectangularClusterTracker::Cluster::getMassNow(int64_t t) {
    float m = mass * std::exp(((float) (lastEventTimestamp - t)) / cfg.clusterMassDecayTauUs);
    return m;
}
// ---------------------------------------------------------------------------
/**
 * Merges information from two source clusters into this cluster to
 * preserve the combined history that is most reliable.
 *
 * @param one
 * @param two
 */
void RectangularClusterTracker::Cluster::mergeTwoClustersToThis(const Cluster& one, const Cluster& two) {
    const Cluster& stronger = one.mass > two.mass ? one : two; // one.firstEventTimestamp < two.firstEventTimestamp ?
    // one : two;
    // Cluster older=one.numEvents>two.numEvents? one:two;
    clusterNumber = stronger.clusterNumber;
    // merge locations by average weighted by mass of events supporting each cluster
    mass = one.mass + two.mass;
    numEvents = one.numEvents + two.numEvents;
    // change to older for location to avoid discontinuities in postion
    location.x = stronger.location.x; // (one.location.x * one.mass + two.location.x * two.mass) / (mass);
    location.y = stronger.location.y; // (one.location.y * one.mass + two.location.y * two.mass) / (mass);

    velocity.x = 0;
    velocity.y = 0;

    angle = stronger.angle;
    cosAngle = stronger.cosAngle;
    sinAngle = stronger.sinAngle;
    averageEventDistance = ((one.averageEventDistance * one.mass) + (two.averageEventDistance * two.mass)) / mass;
    averageEventXDistance = ((one.averageEventXDistance * one.mass) + (two.averageEventXDistance * two.mass)) / mass;
    averageEventYDistance = ((one.averageEventYDistance * one.mass) + (two.averageEventYDistance * two.mass)) / mass;

    lastEventTimestamp = one.lastEventTimestamp > two.lastEventTimestamp ? one.lastEventTimestamp : two.lastEventTimestamp;
    lastUpdateTime = lastEventTimestamp;
    lastPacketLocation.x = stronger.location.x;
    lastPacketLocation.y = stronger.location.y;
    firstEventTimestamp = stronger.firstEventTimestamp; // make lifetime the oldest src cluster
    path = stronger.path;
    birthLocation = stronger.birthLocation;
    // velocityFitter = stronger.velocityFitter;
    velocityPPT.x = stronger.velocityPPT.x;
    velocityPPT.y = stronger.velocityPPT.y;
    velocityPPS.x = stronger.velocityPPS.x;
    velocityPPS.y = stronger.velocityPPS.y;
    velocityValid = stronger.velocityValid;
    vxFilter = stronger.vxFilter;
    vyFilter = stronger.vyFilter;
    avgEventRate = stronger.avgEventRate;
    avgISI = stronger.avgISI;
    hasObtainedSupport = one.hasObtainedSupport || two.hasObtainedSupport; // if either was ever visible then mark merged wasEverVisible
    visibilityFlag = one.visibilityFlag || two.visibilityFlag; // make it visible if either visible
    aspectRatio = stronger.aspectRatio;
    color = stronger.color;
    if (cfg.growMergedSizeEnabled) {
        float R = one.radius + two.radius;
        radius = R + (cfg.mixingFactor * R);
    } else {
        radius = stronger.radius;
    }
}
// ---------------------------------------------------------------------------
void RectangularClusterTracker::Cluster::updatePosition(const PolarityEvent & event, float m) {
    // compute new cluster location by mixing old location with event location by using mixing factor.
    // move the cluster in the direction of the event.
    updatePosition(event.x, event.y, event.x, event.y, m);
}
// ---------------------------------------------------------------------------
#if 0   //TODO: do we have to support this?
void RectangularClusterTracker::Cluster::updatePosition(const ApsDvsOrientationEvent & event) {
    // if event is an orientation event, use the orientation to only move the cluster in a direction
    // perpindicular to the estimated orientation
    ApsDvsOrientationEvent.UnitVector d = OrientationEventInterface.unitVectors[(event.orientation + 2) % 4];
    // calculate projection
    float eventXCentered = event.x - location.x;
    float eventYCentered = event.y - location.y;
    float aDotB = (d.x * eventXCentered) + (d.y * eventYCentered);
    // float aDotA = (d.x * d.x) + (d.y *d.y);
    float division = aDotB; /// aDotA;
    float newX = (division * d.x) + location.x;
    float newY = (division * d.y) + location.y;
    // location.x = (m1 * location.x + m * newX);
    // location.y = (m1 * location.y + m * newY);
    updatePosition(event.x, event.y, newX, newY, m);
}
#endif
// ---------------------------------------------------------------------------
void RectangularClusterTracker::Cluster::updatePosition(float eventX, float eventY, float newX, float newY, float m) {
    float m1 = 1 - m;
    if (!cfg.smoothMove) {
        location.x = ((m1 * location.x) + (m * newX));
        location.y = ((m1 * location.y) + (m * newY));
    } else {
        float errX = (eventX - location.x);
        float errY = (eventY - location.y);

        // float changerate=1/smoothWeight;
        const float m2 = m / cfg.smoothWeight;
        m1 = 1 - m2;

        velocity.x = (m1 * velocity.x) + (m2 * (errX));
        velocity.y = (m1 * velocity.y) + (m2 * (errY));

        location.x = location.x + (velocity.x * cfg.smoothIntegral) + (errX * cfg.smoothPosition);
        location.y = location.y + (velocity.y * cfg.smoothIntegral) + (errX * cfg.smoothPosition);
    }
}
// ---------------------------------------------------------------------------
/**
 * Updates the cluster radius and angle according to distance of event
 * from cluster center, but only if dynamicSizeEnabled or
 * dynamicAspectRatioEnabled or dynamicAngleEnabled.
 *
 * @param event the event to updateShape with
 */
void RectangularClusterTracker::Cluster::updateShape(const PolarityEvent & event) {
    if (cfg.dynamicSizeEnabled) {
        updateSize(event);
    }
    if (cfg.dynamicAspectRatioEnabled) {
        updateAspectRatio(event);
    }
    // PI/2 for vertical positive, -Pi/2 for vertical negative event
    if (cfg.dynamicAngleEnabled) {
        updateAngle(event);
    }

    // turn cluster so that it is aligned along velocity
    if (cfg.angleFollowsVelocity && velocityValid) {
        float velAngle = (float) std::atan2(velocityPPS.y, velocityPPS.x);
        setAngle(velAngle);
    }
}
// ---------------------------------------------------------------------------
void RectangularClusterTracker::Cluster::updateSize(const PolarityEvent & event) {
    float dist = distanceTo(event);
    float oldr = radius;
    float newr = ((1 - cfg.mixingFactor) * oldr) + (dist * cfg.mixingFactor);
    float f;
    if (newr > (f = tracker.defaultClusterRadius * MAX_SCALE_RATIO)) {
        newr = f;
    } else if (newr < (f = tracker.defaultClusterRadius / MAX_SCALE_RATIO)) {
        newr = f;
    }
    setRadius(newr);
}
// ---------------------------------------------------------------------------
/**
 * Returns the implemented distance metric which is the Manhattan
 * distance for speed. This is the sum of abs(dx)+abs(dy).
 *
 * @param dx the x distance
 * @param dy the y distance
 * @return abs(dx)+abs(dy)
 */
float RectangularClusterTracker::Cluster::distanceMetric(float dx, float dy) {
    return ((dx > 0) ? dx : -dx) + ((dy > 0) ? dy : -dy);
}
// ---------------------------------------------------------------------------
void RectangularClusterTracker::Cluster::updateAngle(const PolarityEvent & event) {
    // dynamically rotates cluster to line it up with edge.
    float dx = location.x - event.x;
    float dy = location.y - event.y;
    float newAngle = (float) (std::atan2(dy, dx));
    if (newAngle < 0) {
        newAngle += (float) M_PI; // puts newAngle in 0,PI, e.g -30deg becomes 150deg
    }
    float diff = newAngle - angle;
    if ((diff) > (M_PI / 2)) {
        // newAngle is clockwise a lot, flip it back across to
        // negative value that can be averaged; e.g. instantaneousAngle=10, newAngle=179, newAngle->-1.
        newAngle = newAngle - (float) M_PI;
    } else if (diff < (-M_PI / 2)) {
        // newAngle is CCW
        newAngle = -(float) M_PI + newAngle; // instantaneousAngle=10, newAngle=179, newAngle->1
    }
    float angleDistance = newAngle - angle;
    // makes instantaneousAngle=0 for horizontal positive event, PI for horizontal negative event y=0+eps,x=-1,
    // -PI for y=0-eps, x=-1, //
    // PI/2 for vertical positive, -Pi/2 for vertical negative event
    setAngle(angle + (cfg.mixingFactor * angleDistance));
}
// ---------------------------------------------------------------------------
void RectangularClusterTracker::Cluster::updateAspectRatio(const PolarityEvent & event) {
    // TODO aspect ratio must also account for dynamicAngleEnabled.
    float dx = event.x - location.x;
    float dy = event.y - location.y;
    float dw = (dx * cosAngle) + (dy * sinAngle); // dot dx,dy with unit vector of instantaneousAngle of cluster
    float dh = (-dx * sinAngle) + (dy * cosAngle); // and with normal to unit vector
    float oldAspectRatio = aspectRatio;
    float newAspectRatio = std::abs(dh / dw);
    if (cfg.dynamicAngleEnabled) {
        if (newAspectRatio > ASPECT_RATIO_MAX_DYNAMIC_ANGLE_ENABLED) {
            newAspectRatio = ASPECT_RATIO_MAX_DYNAMIC_ANGLE_ENABLED;
        } else if (newAspectRatio < ASPECT_RATIO_MIN_DYNAMIC_ANGLE_ENABLED) {
            newAspectRatio = ASPECT_RATIO_MIN_DYNAMIC_ANGLE_ENABLED;
        }
    } else {
        if (newAspectRatio > ASPECT_RATIO_MAX_DYNAMIC_ANGLE_DISABLED) {
            newAspectRatio = ASPECT_RATIO_MAX_DYNAMIC_ANGLE_DISABLED;
        } else if (newAspectRatio < ASPECT_RATIO_MIN_DYNAMIC_ANGLE_DISABLED) {
            newAspectRatio = ASPECT_RATIO_MIN_DYNAMIC_ANGLE_DISABLED;
        }
    }
    aspectRatio = ((1 - cfg.mixingFactor) * oldAspectRatio) + (cfg.mixingFactor * newAspectRatio);
}
// ---------------------------------------------------------------------------
void RectangularClusterTracker::Cluster::updateAverageEventDistance(float m) {
    if (std::isnan(averageEventDistance)) {
        ofLog(OF_LOG_WARNING, "distance is NaN");
    }
    float m1 = 1 - m;
    //m is specified when calling this method, it is the mixing factor.
    averageEventDistance = (m1 * averageEventDistance) + (m * distanceToLastEvent);
    averageEventXDistance = (m1 * averageEventXDistance) + (m * xDistanceToLastEvent);
    averageEventYDistance = (m1 * averageEventYDistance) + (m * yDistanceToLastEvent);
}
// ---------------------------------------------------------------------------
void RectangularClusterTracker::Cluster::updateEventRate(const PolarityEvent & event, float m) {
    int64_t prevLastTimestamp = lastEventTimestamp;
    lastEventTimestamp = event.timestamp;
    numEvents++;
    instantaneousISI = lastEventTimestamp - prevLastTimestamp;
    if (instantaneousISI <= 0) {
        instantaneousISI = 1;
    }
    float m1 = 1 - m;
    avgISI = (m1 * avgISI) + (m * instantaneousISI);
    instantaneousEventRate = 1.0f / instantaneousISI;
    avgEventRate = (m1 * avgEventRate) + (m * instantaneousEventRate);
}
// ---------------------------------------------------------------------------
/**
 * Sets the flag of cluster visibility (check is separated from check
 * for efficiency because this operation is costly.) birthLocation and
 * hasObtainedSupport flags are set by this check.
 * <p>
 * A cluster is set visible if both of following are true
 * numEvents>thresholdMassForVisibleCluster AND
 * getMassNow()>thresholdMassForVisibleCluster.
 * <p>
 * Also, if useVelocity is set, then it must be that speed <
 * thresholdVelocityForVisibleCluster
 *
 * @see #isVisible()
 * @see #getMassNow()
 * @param t the current timestamp
 * @return true if cluster is visible
 */
bool RectangularClusterTracker::Cluster::checkAndSetClusterVisibilityFlag(int64_t t) {
    bool ret = true;
    // TODO: In the tooltip it is promised that the thresholdMassForVisibleCluster is
    // checking the MASS of the cluster to determine if its visible. However as far
    // as I see here this is not the case! Instead we check only for the number of Events this cluster has
    // gathered
    if ((numEvents < cfg.thresholdMassForVisibleCluster)
            || ((numEvents > cfg.thresholdMassForVisibleCluster) && (getMassNow(t) < cfg.thresholdMassForVisibleCluster))) {
        ret = false;
    }
    if (cfg.useVelocity) {
        double speed = (std::sqrt((velocityPPT.x * velocityPPT.x) + (velocityPPT.y * velocityPPT.y)) * 1e6)
                / TICK_DEFAULT_US; // speed is in pixels/sec
        if (speed < cfg.thresholdVelocityForVisibleCluster) {
            ret = false;
        }
    }
    if (!hasObtainedSupport && ret) {
        birthLocation.x = location.x;
        birthLocation.y = location.y; // reset location of birth to presumably less noisy current location.
    }
    hasObtainedSupport = (hasObtainedSupport || ret);
    visibilityFlag = ret;
    return ret;
}
// ---------------------------------------------------------------------------
void RectangularClusterTracker::Cluster::setRadius(float r) {
    if (!cfg.highwayPerspectiveEnabled) {
        radius = r;
    } else {
        radius = tracker.defaultClusterRadius * getPerspectiveScaleFactor();
    }
    radiusX = radius / aspectRatio;
    radiusY = radius * aspectRatio;
}
// ---------------------------------------------------------------------------
/**
 * Computes a geometrical updateShape factor based on location of a
 * point relative to the vanishing point. If a vanishingPoint pixel has
 * been selected then we compute the perspective from this vanishing
 * point, otherwise it is the top middle pixel, however this perspective
 * computation is only done if highwayPerspectiveEnabled is true.
 *
 * @return updateShape factor, which assumes a flat surface with
 * vanishing point at vanishingPoint. Size grows linearly to 1 at bottom
 * of scene and shrinks to zero at vanishing point. To sides of scene
 * the size grows again.
 */
float RectangularClusterTracker::Cluster::getPerspectiveScaleFactor() {
    if (!cfg.highwayPerspectiveEnabled) {
        return 1;
    }
    const float MIN_SCALE = 0.1f; // to prevent microclusters that hold only a single pixel
    if (!tracker.vanishingPoint.has_value()) {
        float scale = 1.0f - (location.y / tracker.chipSizeY); // yfrac grows to 1 at bottom of image
        if (scale < MIN_SCALE) {
            scale = MIN_SCALE;
        }
        return scale;
    } else {
        // updateShape is MIN_SCALE at vanishing point or above and grows linearly to 1 at max size of chip
        int size = tracker.chipMaxSize;
        float d = (float) location.distance(tracker.vanishingPoint->x, tracker.vanishingPoint->y);
        float scale = d / size;
        if (scale < MIN_SCALE) {
            scale = MIN_SCALE;
        }
        return scale;
    }
}
// ---------------------------------------------------------------------------
void RectangularClusterTracker::Cluster::setAngle(float angle) {
    if (this->angle != angle) { // save some cycles if unchanged
        this->angle = angle;
        cosAngle = (float) std::cos(angle);
        sinAngle = (float) std::sin(angle);
        initialAngle = ((1 - cfg.mixingFactor) * initialAngle) + (cfg.mixingFactor * angle);
    }
}
// ---------------------------------------------------------------------------
/**
 * Computes and returns speed of cluster in pixels per second.
 *
 * @return speed in pixels per second.
 */
float RectangularClusterTracker::Cluster::getSpeedPPS() const {
    return std::sqrt((velocityPPS.x * velocityPPS.x) + (velocityPPS.y * velocityPPS.y));
}
// ---------------------------------------------------------------------------
/**
 * Computes and returns speed of cluster in pixels per timestamp tick.
 *
 * @return speed in pixels per timestamp tick.
 */
float RectangularClusterTracker::Cluster::getSpeedPPT() const {
    return std::sqrt((velocityPPT.x * velocityPPT.x) + (velocityPPT.y * velocityPPT.y));
}
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
RectangularClusterTracker::FastClusterFinder::FastClusterFinder(int width, int height) {
    nx = width >> SUBSAMPLE_BY;
    ny = height >> SUBSAMPLE_BY;
    grid.resize((nx+1) * (ny+1));
}
// ---------------------------------------------------------------------------
RectangularClusterTracker::Cluster * RectangularClusterTracker::FastClusterFinder::findClusterNear(const PolarityEvent& ev) {
    int i = ((unsigned)ev.x) >> SUBSAMPLE_BY;
    int j = ((unsigned)ev.y) >> SUBSAMPLE_BY;
    return gridLookup(i, j);
}
// ---------------------------------------------------------------------------
void RectangularClusterTracker::FastClusterFinder::update(Cluster * c) {
    removeCluster(c);
    int x = (int) (c->getLocation().x) >> SUBSAMPLE_BY;
    if (x < 0) {
        x = 0;
    } else if (x >= nx) {
        x = nx - 1;
    }
    int y = (int) (c->getLocation().y) >> SUBSAMPLE_BY;
    if (y < 0) {
        y = 0;
    } else if (y >= ny) {
        y = ny - 1;
    }
    gridLookup(x,y) = c;
    map[c] = Point2D<int>(x, y);
}
// ---------------------------------------------------------------------------
void RectangularClusterTracker::FastClusterFinder::reset() {
    std::fill(grid.begin(), grid.end(), nullptr);
}
// ---------------------------------------------------------------------------
void RectangularClusterTracker::FastClusterFinder::removeCluster(Cluster * c) {
    if (auto it = map.find(c); it != map.end()) {
        auto p = it->second;
        gridLookup(p.x,p.y) = nullptr;
        map.erase(c);
    }
}
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
RectangularClusterTracker::RectangularClusterTracker(const Config& cfg, int width, int height)
    : chipSizeX(width)
    , chipSizeY(height)
    , chipMaxSize(std::max(chipSizeX, chipSizeY))
    , cfg(cfg)
    , fastClusterFinder(width, height)
{
    initFilter();
}
// ---------------------------------------------------------------------------
bool RectangularClusterTracker::filter(const PolarityEvent & ev) {

    int sx = chipSizeX;
    int sy = chipSizeY;

    // Update the clusters after the updateInterval, or when there is a jump back in time
    if (ev.timestamp > lastClusterUpdateTime + cfg.updateIntervalMs * 1000 ||
        ev.timestamp < lastClusterUpdateTime) {
        updateClusterList(ev.timestamp);
    }

    if ((ev.x < 0) || (ev.x >= sx) || (ev.y < 0) || (ev.y >= sy)) {
        return false; // out of bounds from e.g. steadicom transform
    }

    Cluster * closest = findClusterNear(ev);

    if (closest != nullptr) {
        closest->addEvent(ev);
    } else if (clusters.size() < cfg.maxNumClusters) { // start a new cluster
        clusters.push_back(std::make_shared<Cluster>(*this, ev));
        closest = clusters.back().get();
    }

    return closest && closest->isVisible();
}
// ---------------------------------------------------------------------------
void RectangularClusterTracker::filter(const PolaritiesQueue & input, PolaritiesQueue & output) {

    output.clear();
    int sx = chipSizeX;
    int sy = chipSizeY;

    // record cluster locations before packet is processed
    for (auto& c : clusters) {
        c->setLastPacketLocation();
    }

    for (const auto & ev : input) {

        if ((ev.x < 0) || (ev.x >= sx) || (ev.y < 0) || (ev.y >= sy)) {
            continue; // out of bounds from e.g. steadicom transform
        }
        Cluster * closest = fastClusterFinder.findClusterNear(ev);

        if (closest != nullptr) {
            closest->addEvent(ev, output);
        } else if (clusters.size() < cfg.maxNumClusters) { // start a new cluster
            clusters.push_back(std::make_shared<Cluster>(*this, ev, output));
        }
    }
}
// ---------------------------------------------------------------------------
/**
 * This method updates the list of clusters, pruning and merging clusters
 * and updating positions based on cluster velocities. It also updates the
 * optical gyro if enabled.
 *
 * @param t the global timestamp of the update.
 */
void RectangularClusterTracker::updateClusterList(int64_t t) {
    lastClusterUpdateTime = t;
    pruneClusters(t);
    mergeClusters();
    updateClusterLocations(t);
    updateClusterPaths(t);
    updateClusterMasses(t);
    visibleClusters.clear();
    for (ClusterPtr& c : clusters) {
        if (c->checkAndSetClusterVisibilityFlag(t)) {
            visibleClusters.push_back(c);
        }
    }
}
// ---------------------------------------------------------------------------
void RectangularClusterTracker::draw(const ofRectangle& stage) {
    ofPushMatrix();
    ofTranslate(stage.getTopLeft());
    ofScale(stage.getWidth() / chipSizeX, stage.getHeight() / chipSizeY);

    ofNoFill();
    for (ClusterPtr& c : clusters) {
        c->draw();
    }

    ofPopMatrix();
}
// ---------------------------------------------------------------------------
RectangularClusterTracker::Cluster * RectangularClusterTracker::findClusterNear(const PolarityEvent& ev) {
    Cluster * c = fastClusterFinder.findClusterNear(ev);
    if (c == nullptr) {
        if (cfg.useNearestCluster) {
            c = getNearestCluster(ev);
        } else {
            c = getFirstContainingCluster(ev); // find cluster that event falls within (or also within surround if scaling enabled)
        }
    }
    return c;
}
// ---------------------------------------------------------------------------
RectangularClusterTracker::Cluster * RectangularClusterTracker::getNearestCluster(const PolarityEvent& event) { // TODO needs to account for the cluster angle
    float minDistance = std::numeric_limits<float>::max();
    Cluster * closest = nullptr;
    float currentDistance = 0;
    for (auto& c : clusters) {
        float rX = c->getRadiusX();
        float rY = c->getRadiusY(); // this is surround region for purposes of dynamicSize scaling of cluster size or
        // aspect ratio
        if (cfg.dynamicSizeEnabled) {
            rX *= cfg.surround;
            rY *= cfg.surround; // the event is captured even when it is in "invisible surround"
        }
        float dx, dy;// TODO use cluster angle here
        if (((dx = c->distanceToX(event)) < rX) && ((dy = c->distanceToY(event)) < rY)) { // TODO needs
            // instantaneousAngle metric
            currentDistance = dx + dy;
            if (currentDistance < minDistance) {
                closest = c.get();
                minDistance = currentDistance;
                c->setDistanceToLastEvent(minDistance, dx, dy); // store data in cluste for later use in moving cluster
            }
        }
    }
    return closest;
}
// ---------------------------------------------------------------------------
RectangularClusterTracker::Cluster * RectangularClusterTracker::getFirstContainingCluster(const PolarityEvent& event) {
    float minDistance = std::numeric_limits<float>::max();
    Cluster * closest = nullptr;
    float currentDistance = 0;
    for (auto& c : clusters) {
        float rX = c->getRadiusX();
        float rY = c->getRadiusY(); // this is surround region for purposes of dynamicSize scaling of cluster size or
        // aspect ratio
        if (cfg.dynamicSizeEnabled) {
            rX *= cfg.surround;
            rY *= cfg.surround; // the event is captured even when it is in "invisible surround"
        }
        float dx, dy;
        if (((dx = c->distanceToX(event)) < rX) && ((dy = c->distanceToY(event)) < rY)) { // TODO needs to account for
            // instantaneousAngle
            currentDistance = dx + dy;
            closest = c.get();
            minDistance = currentDistance;
            c->setDistanceToLastEvent(minDistance, dx, dy); // store data in cluste for later use in moving cluster

            break;
        }
    }
    return closest;
}
// ---------------------------------------------------------------------------
/**
 * Updates cluster locations based on cluster velocities, if
 * {@link #useVelocity} is enabled.
 *
 * @param t the global timestamp of the update.
 */
void RectangularClusterTracker::updateClusterLocations(int64_t t) {
    if (!cfg.useVelocity) {
        return;
    }

    for (ClusterPtr& c : clusters) {
        c->updateLocation(t, averageVelocityPPT);
        fastClusterFinder.update(c.get());
    }
}
// ---------------------------------------------------------------------------
/**
 * Updates cluster path lists and counts number of visible clusters.
 *
 * @param t the update timestamp
 */
void RectangularClusterTracker::updateClusterPaths(int64_t t) {
    // update paths of clusters
    numVisibleClusters = 0;
    for (ClusterPtr& c : clusters) {
        c->updatePath(t);
        if (c->isVisible()) {
            numVisibleClusters++;
        }
    }
}
// ---------------------------------------------------------------------------
void RectangularClusterTracker::updateClusterMasses(int64_t t) {
    for (auto& c : clusters) {
        c->updateMass(t);
    }
}
// ---------------------------------------------------------------------------
/**
 * merge clusters that are too close to each other and that have
 * sufficiently similar velocities (if velocityAngleToRad). this must be
 * done interactively, because feed-forward merging of 4 or more clusters
 * can result in more clusters than you start with. each time we merge two
 * clusters, we start over, until there are no more merges on iteration. for
 * each cluster, if it is close to another cluster then merge them and start
 * over.
 */
void RectangularClusterTracker::mergeClusters() {
    if (cfg.dontMergeEver) {
        return;
    }

    bool mergePending;
    ClusterPtr c1;
    ClusterPtr c2;
    do {
        mergePending = false;
        bool break_outer = false;
        for (auto it1 = clusters.begin(); it1 != clusters.end() && !break_outer; ++it1) {
            c1 = *it1; //clusters[i];
            auto it2 = it1;
            ++it2;
            for (; it2 != clusters.end(); ++it2) {
                c2 = *it2; //clusters[j]; // getString the other cluster
                // final bool overlapping = c1->distanceTo(c2) < (c1->getRadius() + c2->getRadius());
                const bool overlapping = c1->isOverlappingCenterOf(*c2);
                bool velSimilar = true; // start assuming velocities are similar
                if (overlapping && (cfg.velAngDiffDegToNotMerge > 0) && c1->isVisible() && c2->isVisible() && c1->isVelocityValid()
                        && c2->isVelocityValid() && (c1->velocityAngleToRad(*c2) > ((cfg.velAngDiffDegToNotMerge * M_PI) / 180))) {
                    // if velocities valid for both and velocities are sufficiently different
                    velSimilar = false; // then flag them as different velocities
                }
                if (overlapping && velSimilar) {
                    // if cluster is close to another cluster, merge them
                    // if distance is less than sum of radii merge them and if velAngle < threshold
                    mergePending = true;
                    break_outer = true;
                    break; // break out of the outer loop
                }
            }
        }
        if (mergePending && (c1 != nullptr) && (c2 != nullptr)) {
            pruneList.push_back(c1);
            pruneList.push_back(c2);
            clusters.remove(c1);
            clusters.remove(c2);
            fastClusterFinder.removeCluster(c1.get());
            fastClusterFinder.removeCluster(c2.get());
            clusters.push_back(createCluster(c1, c2));
        }
    } while (mergePending);
    // update all cluster sizes
    // note that without this following call, clusters maintain their starting size until they are merged with
    // another cluster.
    if (cfg.highwayPerspectiveEnabled) {
        for (auto& c : clusters) {
            c->setRadius(defaultClusterRadius);
        }
    }
}
// ---------------------------------------------------------------------------
/**
 * Prunes out old clusters that don't have support or that should be purged
 * for some other reason.
 *
 * @param t the timestamp of the purge operation
 */
void RectangularClusterTracker::pruneClusters(int64_t t) {
    pruneList.clear();
    for (ClusterPtr& c : clusters) {
        int64_t t0 = c->getLastEventTimestamp();
        // int t1=ae.getLastTimestamp();
        int64_t timeSinceSupport = t - t0;
        if (timeSinceSupport == 0) {
            continue; // don't kill off cluster spawned from first event
        }
        bool massTooSmall = false;
        int lifetime = c->getLifetime();

        if (t > c->getLastEventTimestamp()) {
            lifetime = t - c->getBirthTime();
        }

        float massThreshold = cfg.thresholdMassForVisibleCluster;
        if (cfg.highwayPerspectiveEnabled) {
            massThreshold *= c->getPerspectiveScaleFactor();
        }
        // do not kill off clusters that were just born or have not lived at least their clusterMassDecayTauUs
        if (((lifetime == 0) || (lifetime >= cfg.clusterMassDecayTauUs)) && (c->getMassNow(t) < massThreshold)) {
            massTooSmall = true;
        }
        bool hitEdge = c->hasHitEdge();
        if ((t0 > t) || massTooSmall || (timeSinceSupport < 0) || hitEdge) {
            // ordinarily, we discard the cluster if it hasn't gotten any support for a while, but we also discard
            // it if there is something funny about the timestamps
            pruneList.push_back(c);
        }
    } // clusters

    // if (logDataEnabled && (clusterLogger != null) && (clusterLoggingMethod == ClusterLoggingMethod.LogClusters)
    //         && !pruneList.isEmpty()) {
    //     clusterLogger.logClusterHistories(pruneList);
    // }

    for (ClusterPtr& c : pruneList) {
        c->onPruning();
    }

    for (ClusterPtr& c : pruneList) {
        clusters.remove(c);
        fastClusterFinder.removeCluster(c.get());
    }
}
// ---------------------------------------------------------------------------
void RectangularClusterTracker::initFilter() {
    defaultClusterRadius = std::max(chipSizeX, chipSizeY) * cfg.clusterSize;
}
// ---------------------------------------------------------------------------
RectangularClusterTracker::ClusterPtr RectangularClusterTracker::createCluster(ClusterPtr c1, ClusterPtr c2) {
    return std::make_shared<Cluster>(*this, *c1, *c2);
}
// ---------------------------------------------------------------------------