// ============================================================
// BFSwarmManager.cpp
// ButterFlight — Multi-agent swarm orchestrator
//
// Leader-follower design: followers duplicate the leader rig,
// flap independently, and derive velocity from the leader
// plus flocking separation forces.
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

static constexpr double kCmToM = 0.01;
static constexpr double kMToCm = 100.0;

// ============================================================
// findRigRoot — walk up from the thorax to the highest ancestor
// whose subtree contains ONLY rig-related nodes (joints, meshes,
// transforms). Stop before climbing into a parent that also holds
// cameras, lights, or other non-rig content.
// ============================================================
static bool subtreeIsRigOnly(const MDagPath& root)
{
    MStatus st;
    MItDag it(MItDag::kDepthFirst, MFn::kInvalid, &st);
    if (st != MS::kSuccess) return false;
    it.reset(root, MItDag::kDepthFirst, MFn::kInvalid);
    for (; !it.isDone(); it.next()) {
        MObject obj = it.currentItem();
        if (obj.hasFn(MFn::kCamera) ||
            obj.hasFn(MFn::kLight)  ||
            obj.hasFn(MFn::kNurbsCurve) ||
            obj.hasFn(MFn::kNurbsSurface)) {
            return false;
        }
    }
    return true;
}

MStatus BFSwarmManager::findRigRoot(const MDagPath& thoraxPath,
                                    MDagPath&       outRigRoot)
{
    MStatus st;
    MDagPath current = thoraxPath;

    while (current.length() > 1) {
        MFnDagNode fn(current, &st);
        if (st != MS::kSuccess) break;
        if (fn.parentCount() == 0) break;

        MObject parentObj = fn.parent(0, &st);
        if (st != MS::kSuccess) break;
        if (parentObj.hasFn(MFn::kWorld)) break;

        MFnDagNode parentFn(parentObj, &st);
        if (st != MS::kSuccess) break;
        MDagPath parentPath;
        st = parentFn.getPath(parentPath);
        if (st != MS::kSuccess) break;

        if (!subtreeIsRigOnly(parentPath)) break;

        current = parentPath;
    }

    outRigRoot = current;
    return MS::kSuccess;
}

// ============================================================
// duplicateRig — duplicate the whole rig group via MEL and
//                resolve the new root joint inside the copy.
// ============================================================
MStatus BFSwarmManager::duplicateRig(const MDagPath& rigRoot,
                                     const MString&  rootLeafName,
                                     MDagPath&       outNewRootJoint)
{
    MStatus st;

    MString rigFullName = rigRoot.fullPathName();

    // duplicate -un: duplicates the rig together with its upstream
    // network (skinCluster, tweak, etc.), so the copied meshes
    // deform from the NEW joints automatically — no rebind needed.
    MString cmd = "duplicate -rr -un \"" + rigFullName + "\"";
    MCommandResult cmdResult;
    st = MGlobal::executeCommand(cmd, cmdResult);
    if (st != MS::kSuccess) {
        MGlobal::displayError("BFSwarmManager: 'duplicate' command failed for '"
                              + rigFullName + "'.");
        return st;
    }

    MStringArray resultNames;
    cmdResult.getResult(resultNames);
    if (resultNames.length() == 0) {
        MGlobal::displayError("BFSwarmManager: duplicate returned no names.");
        return MS::kFailure;
    }

    MString newGroupName = resultNames[0];

    MSelectionList sel;
    st = sel.add(newGroupName);
    if (st != MS::kSuccess) {
        MGlobal::displayError("BFSwarmManager: Cannot find duplicated group '"
                              + newGroupName + "'.");
        return st;
    }

    MDagPath newGroupPath;
    sel.getDagPath(0, newGroupPath);

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
// spawn — create N-1 follower agents from one source rig
//
// The leader (agent 0) is the original rig, managed by
// BFSimulateCmd.  This method only creates followers.
// ============================================================
MStatus BFSwarmManager::spawn(const MString& sourceRootName,
                              int totalAgents)
{
    MStatus st;
    int followerN = totalAgents - 1;
    if (followerN <= 0) {
        m_agents.clear();
        return MS::kSuccess;
    }

    m_agents.clear();
    m_agents.resize(followerN);

    // ---- Resolve source skeleton to get position & rig root -----
    BFSkeleton srcSkel;
    st = BFSimulateCmd::readSkeleton(sourceRootName, srcSkel);
    if (st != MS::kSuccess) return st;

    MFnTransform srcRootFn(srcSkel.joints[kThorax]);
    MVector srcPos = srcRootFn.getTranslation(MSpace::kWorld);  // cm

    MDagPath rigRoot;
    st = findRigRoot(srcSkel.joints[kThorax], rigRoot);
    if (st != MS::kSuccess) return st;

    MFnDagNode sourceRootFn(srcSkel.joints[kThorax]);
    MString rootLeafName = sourceRootFn.name();

    // ---- Random number generator --------------------------------
    // Seed from random_device so each spawn gives a different layout.
    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_real_distribution<double> spreadDist(-spawnSpread, spawnSpread);
    std::uniform_real_distribution<double> phaseDist(-maxPhaseJitter, maxPhaseJitter);

    // Per-agent wander / gain / speed-scale distributions.  Keeping
    // variation modest (±20% speed, seek gain 1.0..2.0) preserves the
    // "same speed as leader" feel while still breaking the lockstep
    // motion that made followers look mechanical.
    std::uniform_real_distribution<double> wanderPhaseDist(0.0, 2.0 * M_PI);
    std::uniform_real_distribution<double> seekGainDist(1.0, 2.0);
    std::uniform_real_distribution<double> speedScaleDist(0.85, 1.15);

    // ---- Duplicate for each follower ----------------------------
    for (int i = 0; i < followerN; ++i) {
        MDagPath newRootJoint;
        st = duplicateRig(rigRoot, rootLeafName, newRootJoint);
        if (st != MS::kSuccess) {
            MGlobal::displayError(
                MString("BFSwarmManager: Failed to create follower ") + i);
            return st;
        }

        MString fullPath = newRootJoint.fullPathName();
        st = BFSimulateCmd::readSkeleton(fullPath, m_agents[i].state.skeleton);
        if (st != MS::kSuccess) return st;

        // Scatter position around the source (cm)
        double ox = spreadDist(rng);
        double oy = spreadDist(rng) * 0.3;  // less vertical scatter
        double oz = spreadDist(rng);

        MPoint spawnPosCm(srcPos.x + ox, srcPos.y + oy, srcPos.z + oz);

        // Store position in metres (physics units)
        m_agents[i].state.position = MPoint(spawnPosCm.x * kCmToM,
                                            spawnPosCm.y * kCmToM,
                                            spawnPosCm.z * kCmToM);

        // Move the duplicated rig to the spawn position (Maya cm)
        MFnTransform newRootFn(newRootJoint);
        newRootFn.setTranslation(MVector(spawnPosCm.x, spawnPosCm.y, spawnPosCm.z),
                                 MSpace::kWorld);

        // Phase jitter so wings are out of sync
        double jitter = phaseDist(rng);
        for (int a = 0; a < BFState::kNumAngles; ++a)
            m_agents[i].state.perAnglePhase[a] += jitter;

        // Per-agent wander phases — 6 independent offsets feed the
        // sum-of-sines wander force in stepFollowers().  Fresh draws
        // per agent mean no two followers share the same trajectory.
        for (int k = 0; k < 6; ++k)
            m_agents[i].wanderPhase[k] = wanderPhaseDist(rng);
        m_agents[i].wanderTime = 0.0;
        m_agents[i].seekGain   = seekGainDist(rng);
        m_agents[i].speedScale = speedScaleDist(rng);
    }

    MGlobal::displayInfo(
        MString("BFSwarmManager: Spawned ") + followerN +
        " followers (spread=" + spawnSpread + " cm).");
    return MS::kSuccess;
}

// ============================================================
// stepFollowers — advance all followers by one physics substep
//
// Each follower:
//   1. Advance wing kinematics (independent flapping)
//   2. Apply joint rotations
//   3. Velocity = leader velocity + flocking separation
//   4. Integrate position
// ============================================================
void BFSwarmManager::stepFollowers(const BFState& leader,
                                   double dt, bool pathMode,
                                   bool hoverMode)
{
    const int N = (int)m_agents.size();
    if (N == 0) return;

    // Build state array for flocking: leader at [0], followers at [1..N]
    std::vector<BFState> allStates(N + 1);
    allStates[0] = leader;
    for (int i = 0; i < N; ++i)
        allStates[i + 1] = m_agents[i].state;

    // Compute flocking accelerations for all (including leader at [0])
    std::vector<MVector> flockAccels = flocking.compute(allStates);

    for (int i = 0; i < N; ++i) {
        BFAgent& agent = m_agents[i];
        int prevCycle = agent.state.flapCycle;

        // 1. Wing flapping
        if (pathMode) {
            // Fixed-rate phase advancement (matches leader's path mode)
            for (int a = 0; a < BFState::kNumAngles; ++a)
                agent.state.perAnglePhase[a] += 2.0 * M_PI
                    * agent.state.perAngleFreq[a] * dt;
            agent.state.phase += dt;
            double cyclePeriod = (agent.state.frequency > 0.0)
                                 ? 1.0 / agent.state.frequency : 1.0;
            if (agent.state.phase >= cyclePeriod) {
                agent.state.phase -= cyclePeriod;
                agent.state.flapCycle++;
            }
            agent.wingModel.updateAnglesOnly(agent.state);
        } else {
            agent.wingModel.update(agent.state, dt);
        }

        // 2. Apply joint rotations
        applyAngles(agent.state.skeleton, agent.state.angles,
                    agent.state.heading);

        // 3. Movement logic — hover branch freezes position.
        if (hoverMode) {
            // Leader isn't moving, so followers shouldn't either.
            // They still flap (step 1) but stay at their spawn
            // position with zero velocity.  Heading is left at
            // whatever spawn() produced (typically 0).
            agent.state.velocity = MVector::zero;
        } else {
            // (a) Seek-leader: direction toward leader, magnitude grows
            //     with distance but saturates at maxSpeed.  Per-agent
            //     seekGain varies this pull so followers converge at
            //     different rates rather than in lockstep.
            const double maxSpd = agent.controller.maxSpeed;
            MVector toLeader(leader.position.x - agent.state.position.x,
                             leader.position.y - agent.state.position.y,
                             leader.position.z - agent.state.position.z);
            double dL = toLeader.length();
            MVector seekVel(0.0, 0.0, 0.0);
            if (dL > 1e-4) {
                double seekMag = std::min(dL * agent.seekGain, maxSpd);
                seekVel = (toLeader / dL) * seekMag;
            }

            // (b) Flocking: treat as velocity offset (already in m/s-ish
            //     units after weights).  dt gate keeps per-substep kick
            //     small so nearby agents don't jitter.
            MVector flockVel = flockAccels[i + 1] * dt;

            // (c) Per-agent wander: sum of two low-frequency sines per
            //     axis with per-agent phase offsets.  Produces smooth,
            //     non-repeating jitter so each follower takes a subtly
            //     different path even under identical flocking inputs.
            //     Vertical amplitude is half the horizontal for a more
            //     natural, ground-tethered feel.
            agent.wanderTime += dt;
            const double wt = agent.wanderTime;
            double wx = std::sin(2.0 * M_PI * 0.37 * wt + agent.wanderPhase[0])
                      + 0.5 * std::sin(2.0 * M_PI * 0.81 * wt + agent.wanderPhase[1]);
            double wy = 0.5 * (std::sin(2.0 * M_PI * 0.29 * wt + agent.wanderPhase[2])
                              + 0.5 * std::sin(2.0 * M_PI * 0.67 * wt + agent.wanderPhase[3]));
            double wz = std::sin(2.0 * M_PI * 0.43 * wt + agent.wanderPhase[4])
                      + 0.5 * std::sin(2.0 * M_PI * 0.91 * wt + agent.wanderPhase[5]);
            // Normalize by the sum-of-amplitudes (1.5) so peak ≈ wanderStrength.
            MVector wanderVel(wx * wanderStrength / 1.5,
                              wy * wanderStrength / 1.5,
                              wz * wanderStrength / 1.5);

            // Compose desired-direction vector.  Direction comes from
            // leader velocity + seek + flocking + wander; its magnitude
            // will be overwritten below to match the leader's scalar
            // speed (with a small per-agent scaling).
            MVector dir = leader.velocity + seekVel * 0.5 + flockVel + wanderVel;

            // Smooth blend direction toward the composed target to
            // avoid velocity snaps when flocking or wander changes.
            const double blendRate = 6.0;
            double alpha = 1.0 - std::exp(-blendRate * dt);
            agent.state.velocity = agent.state.velocity * (1.0 - alpha)
                                 + dir * alpha;

            // Enforce scalar-speed match with the leader, scaled by
            // the per-agent speedScale (±15%).  If the leader is
            // essentially still, followers stop too — mirroring the
            // leader's motion scalar but not its direction.
            double leaderSpeed = leader.velocity.length();
            double targetSpeed = leaderSpeed * agent.speedScale;
            double curSpeed    = agent.state.velocity.length();
            if (targetSpeed < 1e-6) {
                agent.state.velocity = MVector::zero;
            } else if (curSpeed > 1e-9) {
                agent.state.velocity *= targetSpeed / curSpeed;
            }

            // Integrate position
            agent.state.position.x += agent.state.velocity.x * dt;
            agent.state.position.y += agent.state.velocity.y * dt;
            agent.state.position.z += agent.state.velocity.z * dt;

            // Heading from actual velocity direction
            double vx = agent.state.velocity.x;
            double vz = agent.state.velocity.z;
            double hSpeed = std::sqrt(vx * vx + vz * vz);
            if (hSpeed > 0.01)
                agent.state.heading = std::atan2(-vx, -vz);
        }

        // 4. Cycle boundary smoothing
        if (agent.state.flapCycle != prevCycle)
            agent.controller.smoothParameters(agent.state);
    }
}

// ============================================================
// setAgentMaxSpeed — sync every follower's speed cap with leader
// ============================================================
void BFSwarmManager::setAgentMaxSpeed(double maxSpeed)
{
    if (maxSpeed <= 0.0) return;
    for (auto& agent : m_agents) {
        agent.wingModel.maxSpeed  = maxSpeed;
        agent.controller.maxSpeed = maxSpeed;
    }
}

// ============================================================
// writeFollowerKeys — bake keyframes for all followers
// ============================================================
void BFSwarmManager::writeFollowerKeys(const MTime& time)
{
    for (int i = 0; i < (int)m_agents.size(); ++i) {
        const BFAgent& agent = m_agents[i];

        writeAllKeys(agent.state.skeleton, agent.state.angles,
                     agent.state.heading, time);

        // Convert metres → cm for Maya
        MPoint posCm(agent.state.position.x * kMToCm,
                     agent.state.position.y * kMToCm,
                     agent.state.position.z * kMToCm);
        writeTranslationKey(agent.state.skeleton.joints[kThorax],
                            posCm, time);
    }
}

// ============================================================
// clearFollowerAnimCurves — remove stale keys before re-sim
// ============================================================
void BFSwarmManager::clearFollowerAnimCurves()
{
    for (int i = 0; i < (int)m_agents.size(); ++i) {
        const BFSkeleton& skel = m_agents[i].state.skeleton;
        for (int j = 0; j < kNumJoints; ++j) {
            clearAnimCurves(skel.joints[j]);
        }
    }
}

// ============================================================
// Keyframe helpers
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

        curveFn.addKey(time, vals[i], MFnAnimCurve::kTangentLinear,
                       MFnAnimCurve::kTangentLinear, nullptr, &st);
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

        curveFn.addKey(time, vals[i], MFnAnimCurve::kTangentLinear,
                       MFnAnimCurve::kTangentLinear, nullptr, &st);
    }
}

void BFSwarmManager::applyAngles(const BFSkeleton&      skel,
                                 const BFManeuverAngles& ang,
                                 double                  heading)
{
    MStatus st;

    MFnTransform thoraxFn(skel.joints[kThorax], &st);
    if (st == MS::kSuccess) {
        thoraxFn.setRotation(MEulerRotation(
            deg2rad(ang.thetaBeta), heading, 0.0,
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
                                  double                  heading,
                                  const MTime&            time)
{
    // Thorax — pitch + heading (yaw)
    writeRotationKey(skel.joints[kThorax],
        MEulerRotation(deg2rad(ang.thetaBeta), heading, 0.0), time);

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

void BFSwarmManager::clearAnimCurves(const MDagPath& joint)
{
    MStatus st;
    const char* attrs[] = { "translateX", "translateY", "translateZ",
                            "rotateX",    "rotateY",    "rotateZ" };

    MFnDependencyNode depFn(joint.node(), &st);
    if (st != MS::kSuccess) return;

    for (const char* attr : attrs) {
        MPlug plug = depFn.findPlug(attr, true, &st);
        if (st != MS::kSuccess || !plug.isConnected()) continue;

        MPlugArray conns;
        plug.connectedTo(conns, true, false);
        if (conns.length() == 0) continue;

        MObject curveObj = conns[0].node();
        if (!curveObj.hasFn(MFn::kAnimCurve)) continue;

        MFnAnimCurve curveFn(curveObj, &st);
        if (st != MS::kSuccess) continue;

        for (int i = (int)curveFn.numKeys() - 1; i >= 0; --i)
            curveFn.remove(i);
    }
}
