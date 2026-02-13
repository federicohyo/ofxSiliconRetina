// C++ version of RectangularClusterTracker.java
// https://github.com/SensorsINI/jaer/blob/master/src/net/sf/jaer/eventprocessing/tracking/RectangularClusterTracker.java

#pragma once

#include "ofMain.h"
#include "ofxDvsPolarity.hpp"

#include <limits>
#include <deque>
#include <list>
#include <map>
#include <optional>


class RectangularClusterTracker {
public:
    const int chipSizeX;
    const int chipSizeY;
    const int chipMaxSize;

    static constexpr int TICK_DEFAULT_US = 1;

    /**
     * scaling can't make cluster bigger or smaller than this ratio to default
     * cluster size.
     */
    static constexpr float MAX_SCALE_RATIO = 2;

    /**
     * maximum and minimum allowed dynamic aspect ratio when dynamic
     * instantaneousAngle is disabled.
     */
    static constexpr float ASPECT_RATIO_MAX_DYNAMIC_ANGLE_DISABLED = 2.5f, ASPECT_RATIO_MIN_DYNAMIC_ANGLE_DISABLED = 0.5f;

    /**
     * maximum and minimum allowed dynamic aspect ratio when dynamic
     * instantaneousAngle is enabled; then min aspect ratio is set to 1 to make
     * instantaneousAngle point along an edge in the scene.
     */
    static constexpr float ASPECT_RATIO_MAX_DYNAMIC_ANGLE_ENABLED = 1, ASPECT_RATIO_MIN_DYNAMIC_ANGLE_ENABLED = 0.5f;

    // average velocities of all clusters mixed with this factor to produce
    // this "prior" on initial cluster velocityPPT
    static constexpr float AVERAGE_VELOCITY_MIXING_FACTOR = 0.001f;



    using PolarityEvent = ofxDvsPolarity;
    using PolaritiesQueue = std::deque<PolarityEvent>;

    struct Config {
        float/*int*/ updateIntervalMs = 100;        // Update interval of the clusters
        bool filterEnabled = true;

        float/*int*/ maxNumClusters = 10;           // global, "Sets the maximum potential number of clusters"

        bool  useEllipticalClusters = false;        // disp, "true uses elliptical rather than rectangular clusters - distance based on elliptical distance including cluster angle"
        float/*int*/ pathLength = 100;              // disp, "paths are at most this many packets long"
        bool  showClusterNumber = false;            // disp, "shows cluster ID number"
        bool  showClusterEps = false;               // disp, "shows cluster events per second"
        bool  showClusterRadius = false;            // disp, "draws cluster radius"
        bool  showClusterVelocity = false;          // disp, "shows velocity vector as (vx,vy) in px/s"
        bool  showClusterVelocityVector = false;    // disp, "draws velocity in using scaling velocityVectorScaling"
        bool  showClusterMass = false;              // disp, "shows cluster mass; mass is decaying measure of the rate of captured events"
        bool  showPaths = false;                    // disp, "shows the stored path points of each cluster
        float velocityVectorScaling = 1;            // disp, "scaling of drawn velocity vectors from pps to pixels in AEChip pixel space"

        float mixingFactor = 0.05f;                 // mov,  "how much cluster is moved towards an event, as a fraction of the distance from the cluster to the event"
        bool  pathsEnabled = true;                  // mov, "draw paths of clusters over some window"
        bool  useVelocity = true;                   // mov, "uses measured cluster velocity to predict future position; vectors are scaled "
        bool  useNearestCluster = false;            // mov, "event goes to nearest cluster, not to first (usually oldest) cluster containing it"
        float predictiveVelocityFactor = 1;         // mov, "how much cluster position leads position based on estimated velocity"
        bool  initializeVelocityToAverage = false;  // mov, "initializes cluster velocity to moving average of cluster velocities; otherwise initialized to zero"
        float velocityTauMs = 100.0f;               // mov, "lowpass filter time constant in ms for velocity updates; effectively limits acceleration"
        float frictionTauMs = NAN;                  // mov, "velocities decay towards zero with this time constant to mimic friction; set to NaN to disable friction"

        float surround = 2.0f;                      // sizing, "the radius is expanded by this ratio to define events that pull radius of cluster"
        bool  dynamicSizeEnabled = false;           // sizing, "size varies dynamically depending on cluster events"
        bool  dynamicAspectRatioEnabled = false;    // sizing, "aspect ratio of cluster depends on events"
        bool  dynamicAngleEnabled = false;          // sizing, "angle of cluster depends on events, otherwise angle is zero"
        float aspectRatio = 1.0f;                   // sizing, "default (or initial) aspect ratio, <1 is wide"
        float clusterSize = 0.15f;                  // sizing, "size (starting) in fraction of chip max size"
        bool  highwayPerspectiveEnabled = false;    // sizing, "Cluster size depends on perspective location"
        bool  angleFollowsVelocity = false;         // sizing, "cluster angle is set by velocity vector angle; requires that useVelocity is on"

        bool  useOnePolarityOnlyEnabled = false;    // update, "use only one event polarity"
        bool  growMergedSizeEnabled = false;        // update, "enabling makes merged clusters take on sum of sizes, otherwise they take on size of older cluster"
        float velAngDiffDegToNotMerge = 60;         // update, "minimum relative angle in degrees of cluster velocity vectors for which not to merge overlapping clusters. Set this to zero to allow merging independent of cluster velocity. If clusters are moving in different directions, then this will prevent their merging.  The angle should be set at least to 90 deg for this to be effective."

        float/*int*/ thresholdMassForVisibleCluster = 30;   // life, "Cluster needs this \"mass\" to be visible. Mass increments with each event and decays with e-folding time constant of clusterMassDecayTauUs. Use \"showAllClusters\" to diagnose fleeting clusters."
        float thresholdVelocityForVisibleCluster = 0;       // life, "cluster must have at least this velocity in pixels/sec to become visible"
        float/*int*/ clusterMassDecayTauUs = 10000; // life, "time constant of exponential decay of \"mass\" of cluster between events (us)"
        bool  enableClusterExitPurging = true;      // life, "enables rapid purging of clusters that hit edge of scene"
        bool  surroundInhibitionEnabled = false;    // life, "Enabling this option causes events in the surround region to actively reduce the cluster mass, enabling tracking of only isolated features"
        float surroundInhibitionCost = 1.0f;        // life, "If above is checked: The negative weight of surrounding points"
        bool  dontMergeEver = false;                // life, "never merge overlapping clusters"

        bool  smoothMove = false;                   // pi, "Use the PI controller to update particle position and velocity"
        float smoothWeight = 100.0f;                // pi, "If smoothmove is checked, the 'weight' of a cluster"
        float smoothPosition = 0.001f;              // pi, "Position Coefficient"
        float smoothIntegral = 0.001f;              // pi, "Integral Coefficient"
    };


    template <typename T>
    struct Point2D {
        T x;
        T y;

        Point2D() : x{}, y{} {}
        Point2D(T val_x, T val_y) : x(val_x), y(val_y) {}

        template <typename S> void setLocation(const Point2D<S> & other) { x = other.x; y = other.y; }

        double distance(double px, double py) {
            px -= x; py -= y;
            return std::sqrt(px * px + py * py);
        }
    };

    struct ClusterPathPoint : public Point2D<float> {
        /** timestamp of this point. */
        int64_t t;
        /** Number of events that contributed to this point. */
        int nEvents;
        /** Velocity of cluster (filtered) at this point in pixels per timestamp tick (e.g. us).
         * This field is initially null and is initialized by the velocityPPT estimation, if used. */
        Point2D<float> velocityPPT;
        /** disparity of stereo vision. Valid for stereo vision only */
        float stereoDisparity;
        /** Measured size (average radius) of cluster */
        float radiusPixels = -1;
        
        ClusterPathPoint(float x, float y, int64_t t)
            : Point2D<float>(x, y), t(t) {}

        void setnEvents(int nEvents) { this->nEvents = nEvents; }
    };

    class LowpassFilter {
    public:
        void setTauMs(float tau);
        float filter(float val, int64_t time);

    private:
        /** ticks per ms of input time */
        static constexpr int TICK_PER_MS=1000;
        
        /** The filter time constant in ms. Default value is 100ms. */
        float tauMs = 100;
        
        /** The last timestamp used */
        int64_t lastTime = 0;
        
        /** This flag is false until the filter sets it true on the first value. */
        bool initialized = false;

        /** The current state of the filter */
        float lpVal;
        /** The last value */
        float lastVal = 0;
    };

    /**
     * The basic object that is tracked, which is a rectangular cluster with
     * (optionally) variable size, aspect ratio, and angle.
     */
    class Cluster {
        static constexpr float VELPPS_SCALING = 1e6f / TICK_DEFAULT_US;

        RectangularClusterTracker& tracker;
        const Config& cfg;

    public:
        Cluster(RectangularClusterTracker& tracker);
        Cluster(RectangularClusterTracker& tracker, const PolarityEvent& ev);
        Cluster(RectangularClusterTracker& tracker, const PolarityEvent& ev, PolaritiesQueue& output);

        Cluster(RectangularClusterTracker& tracker, const Cluster& one, const Cluster& two);
        void addEvent(const PolarityEvent& ev, PolaritiesQueue& output);
        void addEvent(const PolarityEvent& ev);

        static void drawEllipse(float centerX, float centerY, float radiusX, float radiusY, float angle, int N);
        static void drawCircle(float centerX, float centerY, float radius, int N);
        static void drawBox(float centerX, float centerY, float width, float height, float angle);
        static void drawLine(float startX, float startY, float lengthX, float lengthY, float scale);
        static void drawVector(float origX, float origY, float headX, float headY, float headlength, float scale);

        void draw();
        void onPruning() {}

        void updateMass(int64_t t);
        void updatePath(int64_t t);
        void updateVelocity();
        void updateLocation(int64_t t, Point2D<float>& averageVelocityPPT);
        bool hasHitEdge();

        float distanceTo(const PolarityEvent& event) const;
        float distanceToX(const PolarityEvent& event) const;
        float distanceToY(const PolarityEvent& event) const;
        float distanceTo(const Cluster& c) const;
        float velocityAngleToRad(const Cluster& c) const;

        const Point2D<float>& getLocation() const { return location; }
        float getRadiusX() const { return radiusX; }
        float getRadiusY() const { return radiusY; }

        int64_t getLifetime() const { return lastUpdateTime - firstEventTimestamp; }
        int64_t getBirthTime() const { return firstEventTimestamp;}
        int64_t getLastEventTimestamp() const { return lastEventTimestamp; }

        void setLastPacketLocation() {
            lastPacketLocation.setLocation(location);
        }

        void setDistanceToLastEvent(float minDistance, float dx, float dy) {
            distanceToLastEvent = minDistance;
            xDistanceToLastEvent = dx;
            yDistanceToLastEvent = dy;
        }

        bool isOverlappingCenterOf(const Cluster& c2) const {
            const bool overlapping = distanceTo(c2) < (radius + c2.radius);
            return overlapping;
        }

        bool isVisible() const { return visibilityFlag; }
        bool isVelocityValid() const { return velocityValid; }
        bool isWasEverVisible() { return hasObtainedSupport; }

        float getMassNow(int64_t t);

    private:
        void mergeTwoClustersToThis(const Cluster& one, const Cluster& two);
        void updatePosition(const PolarityEvent & event, float m);

#if 0   //TODO: do we have to support this?
        void updatePosition(const ApsDvsOrientationEvent & event)
#endif
        void updatePosition(float eventX, float eventY, float newX, float newY, float m);
        void updateShape(const PolarityEvent & event);
        void updateSize(const PolarityEvent & event);
        static float distanceMetric(float dx, float dy);
        void updateAngle(const PolarityEvent & event);
        void updateAspectRatio(const PolarityEvent & event);
        void updateAverageEventDistance(float m);\
        void updateEventRate(const PolarityEvent & event, float m);

    public:
        bool checkAndSetClusterVisibilityFlag(int64_t t);
        void setRadius(float r);
        float getPerspectiveScaleFactor();

    private:
        void setAngle(float angle);
        float getSpeedPPS() const;
        float getSpeedPPT() const;

    private:
        Point2D<float> location;            // location in chip pixels
        Point2D<float> velocity;            // velocity of cluster in PPS
        Point2D<float> birthLocation;       // birth location of cluster
        Point2D<float> lastPacketLocation;  // location at end of last packet, used for movement sample
        Point2D<float> velocityPPT;         // velocityPPT of cluster in pixels/tick
        Point2D<float> velocityPPS;         // cluster velocityPPT in pixels/second

        /**
         * Angle of cluster in radians with zero being horizontal and CCW > 0.
         * sinAngle and cosAngle are updated when angle is updated.
         */
        float angle = 0, cosAngle = 1, sinAngle = 0;

        ofFloatColor color;
        int numEvents = 0;                  // number of events collected by this cluster.
        int previousNumEvents = 0;          // total number of events and number at previous packet

        /**
         * First and last timestamp of cluster. <code>firstEventTimestamp</code>
         * is updated when cluster becomes visible.
         * <code>lastEventTimestamp</code> is the last time the cluster was
         * touched either by an event or by some other timestamped update, e.g.
         * null null null null null null null null         {@link #updateClusterList(net.sf.jaer.event.EventPacket, int)
		 * }.
         *
         * @see #isVisible()
         */
        int64_t lastEventTimestamp, firstEventTimestamp;
        
        /**
         * The "mass" of the cluster is the weighted number of events it has
         * collected. The mass decays over time and is incremented by one by
         * each collected event. The mass decays with a first order time
         * constant of clusterMassDecayTauUs in us. If
         * surroundInhibitionEnabled=true, then the mass is decremented by
         * events captured in the surround.
         */
        float mass = 1;

        /**
         * This is the last time in timestamp ticks that the cluster was
         * updated, either by an event or by a regular update such as
         * {@link #updateClusterLocations(int)}. This time can be used to
         * compute position updates given a cluster velocityPPT and time now.
         */
        int64_t lastUpdateTime;
        
        /**
         * events/tick event rate for last two events.
         */
        float instantaneousEventRate; // in events/tick

        /**
         * Flag which is set true (forever) once a cluster has first obtained
         * sufficient support.
         */
        bool hasObtainedSupport = false;

        /**
         * average (mixed using mixingFactor) distance of events from cluster
         * center, a measure of actual cluster size.
         */
        float averageEventDistance, averageEventXDistance, averageEventYDistance;

        int clusterNumber;  // assigned to be the absolute number of the cluster that has been created.

        float avgEventRate = 0;    // Average event rate as computed using mixingFactor
        float radius; // in chip chip pixels
        float aspectRatio, radiusX, radiusY;
        std::deque<ClusterPathPoint> path;

        LowpassFilter vxFilter;
        LowpassFilter vyFilter;
        float avgISI;
        float rgb[4];
        bool  velocityValid = false; // used to flag invalid or uncomputable velocityPPT
        bool  visibilityFlag = false; // this flag updated in updateClusterList
        float instantaneousISI; // ticks/event

        float distanceToLastEvent = std::numeric_limits<float>::infinity();
        float xDistanceToLastEvent = std::numeric_limits<float>::infinity();
        float yDistanceToLastEvent = std::numeric_limits<float>::infinity();

        // Common to all clusters
        static int clusterCounter;
        static float initialAngle;
    };


    class FastClusterFinder {
    public:
        FastClusterFinder(int width, int height);
        Cluster * findClusterNear(const PolarityEvent& ev);
        void update(Cluster * c);
        void reset();
        void removeCluster(Cluster * c);

    private:
        Cluster*& gridLookup(int i, int j) { return grid[i * (ny+1) + j]; }

        static constexpr int SUBSAMPLE_BY = 2;
        std::vector<Cluster*> grid;
        std::map<Cluster*, Point2D<int>> map;
        int nx, ny;
    };

    using ClusterPtr = std::shared_ptr<Cluster>;


    RectangularClusterTracker(const Config& cfg, int width, int height);

    bool filter(const PolarityEvent & ev);
    void filter(const PolaritiesQueue & input, PolaritiesQueue & output);
    void updateClusterList(int64_t t);
    void setVanishingPoint(float x, float y) { vanishingPoint = Point2D<float>(x, y); }
    void resetVanishingPoint() { vanishingPoint.reset(); }
    void draw(const ofRectangle& stage);
    size_t getNumClusters() const { return clusters.size(); }

private:
    Cluster * findClusterNear(const PolarityEvent& ev);
    Cluster * getNearestCluster(const PolarityEvent& event);
    Cluster * getFirstContainingCluster(const PolarityEvent& event);
    void updateClusterLocations(int64_t t);
    void updateClusterPaths(int64_t t);
    void updateClusterMasses(int64_t t);
    void mergeClusters();
    void pruneClusters(int64_t t);

    void initFilter();
    ClusterPtr createCluster(ClusterPtr c1, ClusterPtr c2);

private:
    friend class Cluster;

    const Config& cfg;
    float defaultClusterRadius;
    Point2D<float> averageVelocityPPT;
    std::list<ClusterPtr> clusters;
    std::deque<ClusterPtr> visibleClusters;
    std::deque<ClusterPtr> pruneList;
    int64_t lastClusterUpdateTime = 0;
    int numVisibleClusters = 0;
    FastClusterFinder fastClusterFinder;
    std::optional<Point2D<float>> vanishingPoint;
};