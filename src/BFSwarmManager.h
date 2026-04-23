#pragma once
// ============================================================
// BFSwarmManager.h
// ButterFlight — Multi-agent swarm orchestrator
//
// Leader-follower design: the leader butterfly is simulated
// by BFSimulateCmd (any mode: free flight, path, hover).
// This manager duplicates the rig N-1 times for followers,
// which flap their wings independently but derive velocity
// from the leader plus flocking separation forces.
// ============================================================

#include "BFState.h"
#include "BFWingModel.h"
#include "BFManeuverController.h"
#include "BFFlocking.h"

#include <maya/MString.h>
#include <maya/MStatus.h>
#include <maya/MDagPath.h>
#include <maya/MTime.h>
#include <maya/MPoint.h>
#include <maya/MVector.h>
#include <maya/MEulerRotation.h>
#include <maya/MFnAnimCurve.h>

#include <vector>

// ---- Per-agent bundle: state + its own wing model & controller
//      (each agent has independent phase / frequency / history)
struct BFAgent {
    BFState              state;
    BFWingModel          wingModel;
    BFManeuverController controller;

    // ---- Per-agent randomization (decorrelates followers) ------
    // Seeded once in spawn() so each follower has its own wander
    // trajectory, seek gain, and target-speed variation — so they
    // don't move in lockstep even though they share a leader.
    double wanderTime       = 0.0;                     // accumulated seconds
    double wanderPhase[6]   = { 0,0,0,0,0,0 };         // 3 axes × 2 freqs
    double seekGain         = 1.5;                     // per-agent seek strength
    double speedScale       = 1.0;                     // per-agent scaling of leader speed
};

class BFSwarmManager
{
public:
    // ---- Spawn parameters (set before calling spawn()) ---------
    double  spawnSpread  = 200.0;    // cm — random scatter radius

    // ---- Flocking (public so callers can tweak weights) ---------
    BFFlocking flocking;

    // ---- Phase jitter ------------------------------------------
    double  maxPhaseJitter = 3.14159265358979323846;  // up to +/- PI

    // ---- Wander (organic per-agent jitter) ---------------------
    // Amplitude of the low-frequency sinusoidal wander velocity
    // added to each follower's desired direction.  In the same
    // scale as leader.velocity (m/s) before the scalar-speed
    // normalization that caps magnitude to leader's speed.
    double  wanderStrength = 0.6;

    // ---- Main interface ----------------------------------------

    /// Duplicate the source rig N-1 times (followers only; the
    /// leader is the original rig managed by BFSimulateCmd).
    /// Scatters initial positions around the leader.
    /// @param sourceRootName  Name of the source BF_body joint.
    /// @param totalAgents     Total count including leader.
    /// @return MS::kSuccess if all followers were created.
    MStatus spawn(const MString& sourceRootName, int totalAgents);

    /// Step all followers for one physics substep.
    /// Each follower: advance wing model, set velocity =
    /// leader velocity + flocking separation, integrate position.
    /// @param leader     Current leader state (position in metres).
    /// @param dt         Physics timestep (seconds).
    /// @param pathMode   If true, use fixed-frequency phase advance.
    /// @param hoverMode  If true, followers only flap their wings —
    ///                   velocity is forced to zero and position is
    ///                   not integrated (mirrors the leader's hover).
    void stepFollowers(const BFState& leader, double dt,
                       bool pathMode, bool hoverMode);

    /// Apply a common max-speed cap to every agent's wing model and
    /// controller.  Call after spawn() (and whenever the leader's
    /// cruise speed changes) so followers can't outrun the leader.
    /// @param maxSpeed   Speed cap in m/s.
    void setAgentMaxSpeed(double maxSpeed);

    /// Write keyframes for all followers at the given frame time.
    /// Converts positions from metres to cm for Maya.
    void writeFollowerKeys(const MTime& time);

    /// Clear all anim curves on follower skeletons (call before sim).
    void clearFollowerAnimCurves();

    /// Number of followers (total agents - 1).
    int followerCount() const { return (int)m_agents.size(); }

    /// Read-only access to agents (for testing / inspection).
    const std::vector<BFAgent>& agents() const { return m_agents; }

private:
    std::vector<BFAgent> m_agents;

    // ---- DAG duplication helpers --------------------------------

    static MStatus findRigRoot(const MDagPath& thoraxPath,
                               MDagPath&       outRigRoot);

    static MStatus duplicateRig(const MDagPath& rigRoot,
                                const MString&  rootLeafName,
                                MDagPath&       outNewRootJoint);

    // ---- Keyframe helpers (mirrors BFSimulateCmd statics) -------

    static void applyAngles(const BFSkeleton&      skel,
                            const BFManeuverAngles& ang,
                            double                  heading);

    static void writeAllKeys(const BFSkeleton&      skel,
                             const BFManeuverAngles& ang,
                             double                  heading,
                             const MTime&            time);

    static void writeTranslationKey(const MDagPath& joint,
                                    const MPoint&   pos,
                                    const MTime&    time);

    static void writeRotationKey(const MDagPath&       joint,
                                 const MEulerRotation& rot,
                                 const MTime&          time);

    static MObject ensureAnimCurve(const MDagPath&             joint,
                                   const char*                 attrName,
                                   MFnAnimCurve::AnimCurveType curveType,
                                   MStatus&                    outStatus);

    static void clearAnimCurves(const MDagPath& joint);
};
