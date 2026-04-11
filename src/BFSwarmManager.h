#pragma once
// ============================================================
// BFSwarmManager.h
// ButterFlight — Multi-agent swarm orchestrator
//
// Duplicates a source butterfly rig N times, scatters the
// copies within a spawn spread, and runs the simulation loop
// with flocking forces (separation, alignment, cohesion)
// blended into each agent's maneuvering controller.
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
};

class BFSwarmManager
{
public:
    // ---- Spawn parameters (set before calling spawn()) ---------
    int     agentCount   = 10;       // number of copies to create
    double  spawnSpread  = 200.0;    // cm — random scatter radius

    // ---- Flocking (public so callers can tweak weights) ---------
    BFFlocking flocking;

    // ---- Phase jitter ------------------------------------------
    //  Random per-agent phase offset (in radians) added at spawn
    //  so butterflies don't flap in unison.
    double  maxPhaseJitter = 3.14159265358979323846;  // up to +/- PI

    // ---- Main interface ----------------------------------------

    /// Duplicate the source rig (joint hierarchy + mesh children)
    /// N times and scatter initial positions.  Populates m_agents.
    /// @param sourceRootName  Name of the source BF_thorax joint.
    /// @return MS::kSuccess if all agents were created.
    MStatus spawn(const MString& sourceRootName);

    /// Run the full simulation loop for all agents.
    /// @param mode        Flight mode per agent (1=Free Flight, 2=Path Following, 4=Hover).
    ///                    Flocking forces are added on top of whichever mode is selected.
    /// @param duration    Number of output frames to bake.
    /// @param startFrame  First frame number.
    /// @param flapPeriod  Artist-facing flap period multiplier.
    /// @return MS::kSuccess on completion.
    MStatus simulate(int mode, int duration, int startFrame, double flapPeriod);

    /// Read-only access to agents (for testing / inspection).
    const std::vector<BFAgent>& agents() const { return m_agents; }

private:
    std::vector<BFAgent> m_agents;

    // ---- DAG duplication helpers --------------------------------

    /// Find the top-level group above a thorax joint (e.g. BF_body
    /// or the transform parenting the skeleton + mesh).
    /// Returns the DAG path of the topmost ancestor below the world.
    static MStatus findRigRoot(const MDagPath& thoraxPath,
                               MDagPath&       outRigRoot);

    /// Duplicate the rig root via Maya's "duplicate" command and
    /// resolve the root joint inside the copy.
    /// @param rigRoot           DAG path of the group to duplicate.
    /// @param rootLeafName      Short name of the root joint (e.g. "BF_body").
    /// @param outNewRootJoint   Receives the DAG path of the new root joint.
    /// @return MS::kSuccess on success.
    static MStatus duplicateRig(const MDagPath& rigRoot,
                                const MString&  rootLeafName,
                                MDagPath&       outNewRootJoint);

    // ---- Keyframe helpers (mirrors BFSimulateCmd statics) -------

    static void applyAngles(const BFSkeleton&      skel,
                            const BFManeuverAngles& ang);

    static void writeAllKeys(const BFSkeleton&      skel,
                             const BFManeuverAngles& ang,
                             const MTime&            time);

    static void writeTranslationKey(const MDagPath& joint,
                                    const MPoint&   pos,
                                    const MTime&    time);

    static void writeRotationKey(const MDagPath&       joint,
                                 const MEulerRotation& rot,
                                 const MTime&          time);

    static void writeHeadingKey(const MDagPath& joint,
                                const MVector&  velocity,
                                const MTime&    time);

    static MObject ensureAnimCurve(const MDagPath&             joint,
                                   const char*                 attrName,
                                   MFnAnimCurve::AnimCurveType curveType,
                                   MStatus&                    outStatus);
};
