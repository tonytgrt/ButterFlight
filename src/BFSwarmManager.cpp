// ============================================================
// BFSwarmManager.cpp
// ButterFlight — Multi-agent swarm orchestrator
// ============================================================

#include "BFSwarmManager.h"
#include "BFSimulateCmd.h"

#include <maya/MGlobal.h>
#include <maya/MSelectionList.h>
#include <maya/MDagPath.h>
#include <maya/MFnDagNode.h>
#include <maya/MItDag.h>
#include <maya/MFn.h>
#include <maya/MFnTransform.h>
#include <maya/MEulerRotation.h>
#include <maya/MAnimControl.h>
#include <maya/MTime.h>
#include <maya/MFnAnimCurve.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MCommandResult.h>

#include <cmath>
#include <random>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static inline double deg2rad(double d) { return d * M_PI / 180.0; }

// ============================================================
// findRigRoot — walk up from the thorax to the topmost group
//
// The typical hierarchy is:
//   |world
//     |BF_butterfly_GRP   <-- this is what we want
//       |BF_body
//         |BF_thorax
//         |BF_abdomen
//       |Butterfly_mesh
//
// We walk up until the parent is the world root.
// ============================================================
MStatus BFSwarmManager::findRigRoot(const MDagPath& thoraxPath,
                                    MDagPath&       outRigRoot)
{
    MStatus st;
    MDagPath current = thoraxPath;

    // Walk up until the parent is the world (length == 1 means
    // the node is a direct child of the world root).
    while (current.length() > 1) {
        MFnDagNode fn(current, &st);
        if (st != MS::kSuccess) break;

        if (fn.parentCount() == 0) break;

        MObject parentObj = fn.parent(0, &st);
        if (st != MS::kSuccess) break;

        MDagPath parentPath;
        MFnDagNode parentFn(parentObj, &st);
        if (st != MS::kSuccess) break;
        st = parentFn.getPath(parentPath);
        if (st != MS::kSuccess) break;

        current = parentPath;
    }

    outRigRoot = current;
    return MS::kSuccess;
}

// ============================================================
// duplicateRig — duplicate the whole rig group via MEL and
//                resolve the new thorax joint inside the copy.
//
// Using Maya's native "duplicate -rr" command handles mesh,
// skinCluster, blendShapes, and material connections properly.
// ============================================================
MStatus BFSwarmManager::duplicateRig(const MDagPath& rigRoot,
                                     const MString&  rootLeafName,
                                     MDagPath&       outNewRootJoint)
{
    MStatus st;

    // Build the full DAG path string for the source
    MString rigFullName = rigRoot.fullPathName();

    // Execute: duplicate -rr "rigFullName"
    MString cmd = "duplicate -rr \"" + rigFullName + "\"";
    MCommandResult cmdResult;
    st = MGlobal::executeCommand(cmd, cmdResult);
    if (st != MS::kSuccess) {
        MGlobal::displayError("BFSwarmManager: 'duplicate' command failed for '"
                              + rigFullName + "'.");
        return st;
    }

    // The result is a list of strings — the first is the new group name
    MStringArray resultNames;
    cmdResult.getResult(resultNames);
    if (resultNames.length() == 0) {
        MGlobal::displayError("BFSwarmManager: duplicate returned no names.");
        return MS::kFailure;
    }

    MString newGroupName = resultNames[0];

    // Resolve the new group's DAG path
    MSelectionList sel;
    st = sel.add(newGroupName);
    if (st != MS::kSuccess) {
        MGlobal::displayError("BFSwarmManager: Cannot find duplicated group '"
                              + newGroupName + "'.");
        return st;
    }

    MDagPath newGroupPath;
    sel.getDagPath(0, newGroupPath);

    // Search inside the new group for a joint matching rootLeafName
    MItDag dagIter(MItDag::kDepthFirst, MFn::kJoint, &st);
    if (st != MS::kSuccess) return st;

    st = dagIter.reset(newGroupPath, MItDag::kDepthFirst, MFn::kJoint);
    if (st != MS::kSuccess) return st;

    bool found = false;
    for (; !dagIter.isDone(); dagIter.next()) {
        MDagPath childPath;
        dagIter.getPath(childPath);
        MFnDagNode childFn(childPath);

        if (childFn.name() == rootLeafName) {
            outNewRootJoint = childPath;
            found = true;
            break;
        }
    }

    if (!found) {
        MGlobal::displayError("BFSwarmManager: Could not find '"
                              + rootLeafName + "' in duplicated rig '"
                              + newGroupName + "'.");
        return MS::kFailure;
    }

    return MS::kSuccess;
}

// ============================================================
// spawn — create N agents from one source rig
//
// Agent 0 reuses the original rig.  Agents 1..N-1 are
// duplicated copies scattered around the original position.
// Each agent gets a random phase jitter so they don't flap
// in lockstep.
// ============================================================
MStatus BFSwarmManager::spawn(const MString& sourceRootName)
{
    MStatus st;
    m_agents.clear();
    m_agents.resize(agentCount);

    // ---- Resolve source skeleton --------------------------------
    st = BFSimulateCmd::readSkeleton(sourceRootName, m_agents[0].state.skeleton);
    if (st != MS::kSuccess) return st;

    // Read the source thorax world position
    MFnTransform srcRootFn(m_agents[0].state.skeleton.joints[kThorax]);
    MVector srcPos = srcRootFn.getTranslation(MSpace::kWorld);
    m_agents[0].state.position = MPoint(srcPos.x, srcPos.y, srcPos.z);

    // ---- Find the topmost rig group for duplication -------------
    MDagPath rigRoot;
    st = findRigRoot(m_agents[0].state.skeleton.joints[kThorax], rigRoot);
    if (st != MS::kSuccess) return st;

    // ---- Random number generator --------------------------------
    std::mt19937 rng(42);  // fixed seed for reproducibility
    std::uniform_real_distribution<double> spreadDist(-spawnSpread, spawnSpread);
    std::uniform_real_distribution<double> phaseDist(-maxPhaseJitter, maxPhaseJitter);

    // ---- Phase jitter for agent 0 --------------------------------
    double jitter0 = phaseDist(rng);
    for (int a = 0; a < BFState::kNumAngles; ++a)
        m_agents[0].state.perAnglePhase[a] += jitter0;

    // ---- Duplicate for agents 1..N-1 ----------------------------
    //  Use the leaf name of whatever the user passed as the source
    //  root (e.g. "BF_body"), not hardcoded "BF_thorax".
    MFnDagNode sourceRootFn(m_agents[0].state.skeleton.joints[kThorax]);
    MString rootLeafName = sourceRootFn.name();

    for (int i = 1; i < agentCount; ++i) {
        MDagPath newRootJoint;
        st = duplicateRig(rigRoot, rootLeafName, newRootJoint);
        if (st != MS::kSuccess) {
            MGlobal::displayError(
                MString("BFSwarmManager: Failed to create agent ") + i);
            return st;
        }

        // Use the FULL DAG path so readSkeleton can resolve the
        // duplicated joint unambiguously (short names collide).
        MString fullPath = newRootJoint.fullPathName();
        st = BFSimulateCmd::readSkeleton(fullPath, m_agents[i].state.skeleton);
        if (st != MS::kSuccess) return st;

        // Scatter position around the source
        double ox = spreadDist(rng);
        double oy = spreadDist(rng) * 0.3;  // less vertical scatter
        double oz = spreadDist(rng);

        MPoint spawnPos(srcPos.x + ox, srcPos.y + oy, srcPos.z + oz);
        m_agents[i].state.position = spawnPos;

        // Move the duplicated rig to the spawn position
        MFnTransform newRootFn(newRootJoint);
        newRootFn.setTranslation(MVector(spawnPos.x, spawnPos.y, spawnPos.z),
                                 MSpace::kWorld);

        // Phase jitter so wings are out of sync
        double jitter = phaseDist(rng);
        for (int a = 0; a < BFState::kNumAngles; ++a)
            m_agents[i].state.perAnglePhase[a] += jitter;
    }

    MGlobal::displayInfo(
        MString("BFSwarmManager: Spawned ") + agentCount +
        " agents (spread=" + spawnSpread + " cm).");
    return MS::kSuccess;
}

// ============================================================
// simulate — run the multi-agent simulation loop
//
// For each output frame:
//   1. Compute flocking accelerations for all agents (O(N²))
//   2. For each agent, run substeps:
//      a. Advance wing kinematics (BFWingModel)
//      b. Apply joint rotations (applyAngles)
//      c. Integrate forces + flocking accel (BFManeuverController)
//      d. Sliding-window smoother at cycle boundaries
//   3. Bake keyframes for all agents
// ============================================================
MStatus BFSwarmManager::simulate(int mode, int duration, int startFrame,
                                 double flapPeriod)
{
    if (m_agents.empty()) {
        MGlobal::displayError("BFSwarmManager: No agents. Call spawn() first.");
        return MS::kFailure;
    }

    const int N = (int)m_agents.size();
    const bool isHover = (mode == 4);

    // ---- Timing setup (mirrors BFSimulateCmd) -------------------
    double fps = MTime(1.0, MTime::kSeconds).as(MTime::uiUnit());
    if (fps <= 0.0) fps = 24.0;

    if (flapPeriod <= 0.0) flapPeriod = 1.0;
    double f_gamma_default = m_agents[0].state.perAngleFreq[kAngleGamma];
    double simRate = 1.0 / (f_gamma_default * flapPeriod);

    static constexpr double kTargetSimHz = 960.0;
    int    substeps = std::max(1, (int)std::ceil(kTargetSimHz / fps));
    double simDt = simRate / (fps * substeps);

    // ---- Initialise per-agent controllers -----------------------
    for (int i = 0; i < N; ++i) {
        m_agents[i].controller.maxSpeed = m_agents[i].wingModel.maxSpeed;
    }

    const char* modeNames[] = { "", "Free Flight", "Path Following", "", "Hover" };
    const char* modeName = (mode >= 1 && mode <= 4) ? modeNames[mode] : "Unknown";
    MGlobal::displayInfo(
        MString("BFSwarmManager: Simulating ") + N + " agents, mode=" +
        modeName + ", " + duration + " frames, simDt=" + simDt + "s");

    // ---- Collect state references for flocking ------------------
    std::vector<BFState> stateSnapshot(N);

    // ---- Simulation loop ----------------------------------------
    for (int f = startFrame; f < startFrame + duration; ++f) {

        for (int s = 0; s < substeps; ++s) {

            // Snapshot current states for flocking computation
            for (int i = 0; i < N; ++i)
                stateSnapshot[i] = m_agents[i].state;

            // Compute flocking accelerations (uses snapshot)
            std::vector<MVector> flockAccels = flocking.compute(stateSnapshot);

            // Step each agent
            for (int i = 0; i < N; ++i) {
                BFAgent& agent = m_agents[i];
                int prevCycle = agent.state.flapCycle;

                // 1. Advance wing kinematics (all modes need flapping)
                agent.wingModel.update(agent.state, simDt);

                // 2. Apply rotations so aerodynamics reads current normals
                applyAngles(agent.state.skeleton, agent.state.angles);

                // 3. Physics integration depends on flight mode
                if (isHover) {
                    // Hover: no body forces, only flocking keeps group coherent.
                    // Apply flocking as gentle position drift.
                    agent.state.velocity = flockAccels[i] * simDt;

                    double speed = agent.state.velocity.length();
                    if (speed > agent.controller.maxSpeed * 0.3) {
                        agent.state.velocity *= (agent.controller.maxSpeed * 0.3) / speed;
                    }

                    agent.state.position.x += agent.state.velocity.x * simDt;
                    agent.state.position.y += agent.state.velocity.y * simDt;
                    agent.state.position.z += agent.state.velocity.z * simDt;
                } else {
                    // Free Flight (1) / Path Following (2):
                    // full physics via controller + flocking on top.
                    agent.controller.step(agent.state, simDt);

                    // Apply flocking acceleration (explicit Euler)
                    agent.state.velocity += flockAccels[i] * simDt;

                    // Re-clamp speed after flocking contribution
                    double speed = agent.state.velocity.length();
                    if (speed > agent.controller.maxSpeed) {
                        agent.state.velocity *= agent.controller.maxSpeed / speed;
                    }
                }

                // 4. Sliding-window smoother at cycle boundary
                if (agent.state.flapCycle != prevCycle)
                    agent.controller.smoothParameters(agent.state);
            }
        }

        // ---- Bake keyframes for all agents at this frame --------
        MTime frameTime((double)f, MTime::uiUnit());

        for (int i = 0; i < N; ++i) {
            const BFAgent& agent = m_agents[i];

            writeAllKeys(agent.state.skeleton, agent.state.angles, frameTime);
            writeTranslationKey(agent.state.skeleton.joints[kThorax],
                                agent.state.position, frameTime);
            if (!isHover) {
                writeHeadingKey(agent.state.skeleton.joints[kThorax],
                                agent.state.velocity, frameTime);
            }
        }
    }

    // ---- Set playback range -------------------------------------
    MAnimControl::setMinTime(MTime((double)startFrame, MTime::uiUnit()));
    MAnimControl::setMaxTime(
        MTime((double)(startFrame + duration - 1), MTime::uiUnit()));

    // ---- Report -------------------------------------------------
    MGlobal::displayInfo(
        MString("BFSwarmManager: Baked ") + duration + " frames for " +
        N + " agents (" + startFrame + "-" +
        (startFrame + duration - 1) + "), mode=" + modeName + ".");

    return MS::kSuccess;
}

// ============================================================
// Keyframe helpers — mirrors the static functions in
// BFSimulateCmd.cpp.  Duplicated here to avoid exposing
// those statics in the header.
// ============================================================

MObject BFSwarmManager::ensureAnimCurve(const MDagPath& joint,
                                        const char*     attrName,
                                        MFnAnimCurve::AnimCurveType curveType,
                                        MStatus&        outStatus)
{
    MFnDependencyNode depFn(joint.node(), &outStatus);
    if (outStatus != MS::kSuccess) return MObject::kNullObj;

    MPlug plug = depFn.findPlug(attrName, true, &outStatus);
    if (outStatus != MS::kSuccess) return MObject::kNullObj;

    if (plug.isConnected()) {
        MPlugArray conns;
        plug.connectedTo(conns, true, false);
        if (conns.length() > 0) {
            MObject curveObj = conns[0].node();
            if (curveObj.hasFn(MFn::kAnimCurve)) {
                outStatus = MS::kSuccess;
                return curveObj;
            }
        }
    }

    MFnAnimCurve curveFn;
    MObject curveObj = curveFn.create(plug, curveType, nullptr, &outStatus);
    return curveObj;
}

void BFSwarmManager::writeRotationKey(const MDagPath&       joint,
                                      const MEulerRotation& rot,
                                      const MTime&          time)
{
    MStatus st;
    const char* attrs[3] = { "rotateX", "rotateY", "rotateZ" };
    double      vals [3] = { rot.x,     rot.y,     rot.z     };

    for (int i = 0; i < 3; ++i) {
        MObject curveObj = ensureAnimCurve(joint, attrs[i],
                                          MFnAnimCurve::kAnimCurveTA, st);
        if (st != MS::kSuccess || curveObj.isNull()) continue;

        MFnAnimCurve curveFn(curveObj, &st);
        if (st != MS::kSuccess) continue;

        curveFn.addKey(time, vals[i], MFnAnimCurve::kTangentAuto,
                       MFnAnimCurve::kTangentAuto, nullptr, &st);
    }
}

void BFSwarmManager::writeTranslationKey(const MDagPath& joint,
                                         const MPoint&   pos,
                                         const MTime&    time)
{
    MStatus st;
    const char* attrs[3] = { "translateX", "translateY", "translateZ" };
    double      vals [3] = { pos.x,        pos.y,        pos.z       };

    for (int i = 0; i < 3; ++i) {
        MObject curveObj = ensureAnimCurve(joint, attrs[i],
                                          MFnAnimCurve::kAnimCurveTL, st);
        if (st != MS::kSuccess || curveObj.isNull()) continue;

        MFnAnimCurve curveFn(curveObj, &st);
        if (st != MS::kSuccess) continue;

        curveFn.addKey(time, vals[i], MFnAnimCurve::kTangentAuto,
                       MFnAnimCurve::kTangentAuto, nullptr, &st);
    }
}

void BFSwarmManager::writeHeadingKey(const MDagPath& joint,
                                     const MVector&  velocity,
                                     const MTime&    time)
{
    // Derive heading (rotateY) from velocity direction in the XZ plane
    double speed = velocity.length();
    if (speed < 1.0e-8) return;

    double headingY = std::atan2(velocity.x, velocity.z);  // radians

    MStatus st;
    MObject curveObj = ensureAnimCurve(joint, "rotateY",
                                      MFnAnimCurve::kAnimCurveTA, st);
    if (st != MS::kSuccess || curveObj.isNull()) return;

    MFnAnimCurve curveFn(curveObj, &st);
    if (st != MS::kSuccess) return;

    curveFn.addKey(time, headingY, MFnAnimCurve::kTangentAuto,
                   MFnAnimCurve::kTangentAuto, nullptr, &st);
}

void BFSwarmManager::applyAngles(const BFSkeleton&      skel,
                                 const BFManeuverAngles& ang)
{
    MStatus st;

    MFnTransform thoraxFn(skel.joints[kThorax], &st);
    if (st == MS::kSuccess) {
        thoraxFn.setRotation(MEulerRotation(
            deg2rad(ang.thetaBeta), 0.0, 0.0,
            MEulerRotation::kXYZ));
    }

    MFnTransform fwlFn(skel.joints[kForewingL], &st);
    if (st == MS::kSuccess) {
        fwlFn.setRotation(MEulerRotation(
            deg2rad(ang.thetaZeta),
            deg2rad(ang.thetaPsi),
            deg2rad(ang.thetaGamma),
            MEulerRotation::kXYZ));
    }

    MFnTransform fwrFn(skel.joints[kForewingR], &st);
    if (st == MS::kSuccess) {
        fwrFn.setRotation(MEulerRotation(
            deg2rad(ang.thetaZeta),
            deg2rad(ang.thetaPsi),
            deg2rad(ang.thetaGamma),
            MEulerRotation::kXYZ));
    }

    MFnTransform hwlFn(skel.joints[kHindwingL], &st);
    if (st == MS::kSuccess) {
        hwlFn.setRotation(MEulerRotation(
            0.0, 0.0, deg2rad(ang.thetaGamma),
            MEulerRotation::kXYZ));
    }

    MFnTransform hwrFn(skel.joints[kHindwingR], &st);
    if (st == MS::kSuccess) {
        hwrFn.setRotation(MEulerRotation(
            0.0, 0.0, deg2rad(ang.thetaGamma),
            MEulerRotation::kXYZ));
    }

    MFnTransform abdFn(skel.joints[kAbdomen], &st);
    if (st == MS::kSuccess) {
        abdFn.setRotation(MEulerRotation(
            deg2rad(ang.thetaPhi), 0.0, 0.0,
            MEulerRotation::kXYZ));
    }
}

void BFSwarmManager::writeAllKeys(const BFSkeleton&      skel,
                                  const BFManeuverAngles& ang,
                                  const MTime&            time)
{
    writeRotationKey(skel.joints[kThorax],
        MEulerRotation(deg2rad(ang.thetaBeta), 0.0, 0.0), time);

    writeRotationKey(skel.joints[kForewingL],
        MEulerRotation(deg2rad(ang.thetaZeta),
                       deg2rad(ang.thetaPsi),
                       deg2rad(ang.thetaGamma)), time);

    writeRotationKey(skel.joints[kForewingR],
        MEulerRotation(deg2rad(ang.thetaZeta),
                       deg2rad(ang.thetaPsi),
                       deg2rad(ang.thetaGamma)), time);

    writeRotationKey(skel.joints[kHindwingL],
        MEulerRotation(0.0, 0.0, deg2rad(ang.thetaGamma)), time);

    writeRotationKey(skel.joints[kHindwingR],
        MEulerRotation(0.0, 0.0, deg2rad(ang.thetaGamma)), time);

    writeRotationKey(skel.joints[kAbdomen],
        MEulerRotation(deg2rad(ang.thetaPhi), 0.0, 0.0), time);
}
