// ============================================================
// BFSimulateCmd.cpp
// ButterFlight — Main simulation command implementation
// ============================================================

#include "BFSimulateCmd.h"
#include "BFWingModel.h"
#include "BFManeuverController.h"

#include <maya/MGlobal.h>
#include <maya/MSelectionList.h>
#include <maya/MDagPath.h>
#include <maya/MFnDagNode.h>
#include <maya/MItDag.h>
#include <maya/MFn.h>
#include <maya/MArgList.h>
#include <maya/MFnTransform.h>
#include <maya/MEulerRotation.h>
#include <maya/MAnimControl.h>
#include <maya/MTime.h>
#include <maya/MFnAnimCurve.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MFnDependencyNode.h>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static inline double deg2rad(double d) { return d * M_PI / 180.0; }

// ---- Command name registered with Maya ---------------------
const char* BFSimulateCmd::kCommandName = "bfSimulate";

// ---- Flag constants ----------------------------------------
static const char* kRigFlag      = "-r";
static const char* kRigFlagLong  = "-rigRoot";
static const char* kModeFlag       = "-m";
static const char* kModeFlagLong   = "-mode";
static const char* kDurFlag        = "-d";
static const char* kDurFlagLong    = "-duration";
// NOTE: -frameRate flag kept for backwards compatibility but is
// ignored — FPS is now read directly from Maya's scene time unit
// to prevent mismatches between baked keyframes and playback rate.
static const char* kFpsFlag        = "-f";
static const char* kFpsFlagLong    = "-frameRate";
static const char* kStartFlag      = "-s";
static const char* kStartFlagLong  = "-startFrame";
static const char* kFlapPeriodFlag     = "-fp";
static const char* kFlapPeriodFlagLong = "-flapPeriod";

// ============================================================
// newSyntax — declare accepted flags
// ============================================================
MSyntax BFSimulateCmd::newSyntax()
{
    MSyntax syntax;
    syntax.addFlag(kRigFlag,  kRigFlagLong,  MSyntax::kString);
    syntax.addFlag(kModeFlag,  kModeFlagLong,  MSyntax::kLong);
    syntax.addFlag(kDurFlag,   kDurFlagLong,   MSyntax::kLong);
    syntax.addFlag(kFpsFlag,   kFpsFlagLong,   MSyntax::kDouble);
    syntax.addFlag(kStartFlag, kStartFlagLong, MSyntax::kLong);
    syntax.addFlag(kFlapPeriodFlag, kFlapPeriodFlagLong, MSyntax::kDouble);
    // TODO: add remaining flags (mass, wingArea, gains, eta, etc.)
    return syntax;
}
 
// ============================================================
// readSkeleton — resolve the joint hierarchy from a root name
// ============================================================
MStatus BFSimulateCmd::readSkeleton(const MString& rootJointName,
                                    BFSkeleton&    outSkeleton)
{
    MStatus status;
    outSkeleton.valid = false; 

    // ---- 1. Locate the root joint in the scene ---------------
    MSelectionList selList;
    status = selList.add(rootJointName);
    if (status != MS::kSuccess) {
        MGlobal::displayError(
            "ButterFlight: Root joint '" + rootJointName +
            "' not found in the scene.");
        return MS::kFailure;
    }

    MDagPath rootPath;
    status = selList.getDagPath(0, rootPath);
    if (status != MS::kSuccess) {
        MGlobal::displayError(
            "ButterFlight: Could not obtain DAG path for '" +
            rootJointName + "'.");
        return MS::kFailure;
    }

    // Verify the node is actually a joint
    if (!rootPath.hasFn(MFn::kJoint)) {
        MGlobal::displayError(
            "ButterFlight: '" + rootJointName + "' is not a joint node.");
        return MS::kFailure;
    }

    // Store the thorax (root) joint
    outSkeleton.joints[kThorax] = rootPath;

    // ---- 2. Walk up to the parent of BF_thorax ---------------
    //  In the actual rig, BF_abdomen is a sibling of BF_thorax
    //  (both children of BF_body), not a descendant.  Starting
    //  the search from the parent ensures we find all joints.
    MDagPath searchRoot = rootPath;
    MFnDagNode rootDagFn(rootPath, &status);
    if (status == MS::kSuccess && rootDagFn.parentCount() > 0) {
        MObject parentObj = rootDagFn.parent(0, &status);
        if (status == MS::kSuccess) {
            MFnDagNode parentFn(parentObj, &status);
            if (status == MS::kSuccess) {
                MDagPath parentPath;
                status = parentFn.getPath(parentPath);
                if (status == MS::kSuccess)
                    searchRoot = parentPath;
            }
        }
    }

    // ---- 3. Build a name-to-index map for expected joints ----
    struct JointMapping {
        const char* name;
        BFJointId   id;
    };

    const JointMapping expected[] = {
        { BFJointNames::kForewingL,  kForewingL },
        { BFJointNames::kForewingR,  kForewingR },
        { BFJointNames::kHindwingL,  kHindwingL },
        { BFJointNames::kHindwingR,  kHindwingR },
        { BFJointNames::kAbdomen,    kAbdomen   },
    };
    constexpr int kExpectedCount = sizeof(expected) / sizeof(expected[0]);

    // Track which joints we still need to find
    bool found[kExpectedCount] = {};

    // ---- 4. Depth-first traversal from the parent ------------
    //  Starting from the parent of BF_thorax (typically BF_body)
    //  so we find both children (wings) and siblings (abdomen).
    MItDag dagIter(MItDag::kDepthFirst, MFn::kJoint, &status);
    if (status != MS::kSuccess) {
        MGlobal::displayError(
            "ButterFlight: Failed to create DAG iterator.");
        return MS::kFailure;
    }
    status = dagIter.reset(searchRoot, MItDag::kDepthFirst, MFn::kJoint);
    if (status != MS::kSuccess) {
        MGlobal::displayError(
            "ButterFlight: Failed to reset DAG iterator to search root.");
        return MS::kFailure;
    }

    int foundCount = 0;

    for (; !dagIter.isDone(); dagIter.next()) {
        MDagPath childPath;
        status = dagIter.getPath(childPath);
        if (status != MS::kSuccess) continue;

        MFnDagNode childFn(childPath, &status);
        if (status != MS::kSuccess) continue;

        MString childName = childFn.name();

        // Compare against every expected joint name
        for (int i = 0; i < kExpectedCount; ++i) {
            if (!found[i] && childName == MString(expected[i].name)) {
                outSkeleton.joints[expected[i].id] = childPath;
                found[i] = true;
                ++foundCount;
                break;
            }
        }

        // Early exit once we have found everything
        if (foundCount == kExpectedCount) break;
    }

    // ---- 4. Report results -----------------------------------
    for (int i = 0; i < kExpectedCount; ++i) {
        if (!found[i]) {
            MGlobal::displayWarning(
                MString("ButterFlight: Expected joint '") +
                expected[i].name + "' not found under '" +
                rootJointName + "'.");
        }
    }

    if (foundCount < kExpectedCount) {
        MGlobal::displayError(
            "ButterFlight: Incomplete skeleton — found " +
            MString("") + foundCount + " of " + kExpectedCount +
            " expected child joints.");
        return MS::kFailure;
    }

    outSkeleton.valid = true;
    MGlobal::displayInfo(
        "ButterFlight: Skeleton loaded successfully from '" +
        rootJointName + "'.");
    return MS::kSuccess;
} 

// ============================================================
// applyAngles — set joint rotations from maneuvering angles
//
// Axis mapping (assumes Maya default Y-up, Z-forward rig):
//   Thorax  (BF_thorax):     pitch = rotateX
//   Forewing L/R:            flap  = rotateZ, feather = rotateX, sweep = rotateY
//   Hindwing L/R:            flap  = rotateZ  (1 DOF only)
//   Abdomen (BF_abdomen):    pitch = rotateX
//
// Left/right mirroring: flap (Z) and sweep (Y) are negated for
// the right side so bilateral wings move symmetrically.
// ============================================================
static void applyAngles(const BFSkeleton&       skel,
                        const BFManeuverAngles&  ang)
{
    MStatus st;

    // Thorax pitch
    MFnTransform thoraxFn(skel.joints[kThorax], &st);
    if (st == MS::kSuccess) {
        thoraxFn.setRotation(MEulerRotation(
            deg2rad(ang.thetaBeta), 0.0, 0.0,
            MEulerRotation::kXYZ));
    }

    // Forewing L — flap(Z), feather(X), sweep(Y)
    MFnTransform fwlFn(skel.joints[kForewingL], &st);
    if (st == MS::kSuccess) {
        fwlFn.setRotation(MEulerRotation(
            deg2rad(ang.thetaZeta),
            deg2rad(ang.thetaPsi),
            deg2rad(ang.thetaGamma),
            MEulerRotation::kXYZ));
    }

    // Forewing R — same angles (mirrorBehavior joints handle mirroring)
    MFnTransform fwrFn(skel.joints[kForewingR], &st);
    if (st == MS::kSuccess) {
        fwrFn.setRotation(MEulerRotation(
            deg2rad(ang.thetaZeta),
            deg2rad(ang.thetaPsi),
            deg2rad(ang.thetaGamma),
            MEulerRotation::kXYZ));
    }

    // Hindwing L — flap only (1 DOF)
    MFnTransform hwlFn(skel.joints[kHindwingL], &st);
    if (st == MS::kSuccess) {
        hwlFn.setRotation(MEulerRotation(
            0.0, 0.0, deg2rad(ang.thetaGamma),
            MEulerRotation::kXYZ));
    }

    // Hindwing R — same angle (mirrorBehavior)
    MFnTransform hwrFn(skel.joints[kHindwingR], &st);
    if (st == MS::kSuccess) {
        hwrFn.setRotation(MEulerRotation(
            0.0, 0.0, deg2rad(ang.thetaGamma),
            MEulerRotation::kXYZ));
    }

    // Abdomen rotation (opposite phase to wings, encoded in phi_p = -180)
    MFnTransform abdFn(skel.joints[kAbdomen], &st);
    if (st == MS::kSuccess) {
        abdFn.setRotation(MEulerRotation(
            deg2rad(ang.thetaPhi), 0.0, 0.0,
            MEulerRotation::kXYZ));
    }
}

// ============================================================
// ensureAnimCurve — find or create an animCurveTL/TA on a plug
// ============================================================
static MObject ensureAnimCurve(const MDagPath& joint,
                               const char*     attrName,
                               MFnAnimCurve::AnimCurveType curveType,
                               MStatus&        outStatus)
{
    MFnDependencyNode depFn(joint.node(), &outStatus);
    if (outStatus != MS::kSuccess) return MObject::kNullObj;

    MPlug plug = depFn.findPlug(attrName, true, &outStatus);
    if (outStatus != MS::kSuccess) return MObject::kNullObj;

    // If the plug is already connected to an anim curve, reuse it
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

    // Create a new anim curve
    MFnAnimCurve curveFn;
    MObject curveObj = curveFn.create(plug, curveType, nullptr, &outStatus);
    return curveObj;
}

// ============================================================
// writeRotationKey — set one (X,Y,Z) rotation keyframe
// ============================================================
static void writeRotationKey(const MDagPath&        joint,
                             const MEulerRotation&  rot,
                             const MTime&           time)
{
    MStatus st;
    const char* attrs[3] = { "rotateX", "rotateY", "rotateZ" };
    double      vals [3] = { rot.x,     rot.y,     rot.z     };  // already in radians

    for (int i = 0; i < 3; ++i) {
        MObject curveObj = ensureAnimCurve(joint, attrs[i],
                                          MFnAnimCurve::kAnimCurveTA, st);
        if (st != MS::kSuccess || curveObj.isNull()) continue;

        MFnAnimCurve curveFn(curveObj, &st);
        if (st != MS::kSuccess) continue;

        // addKey uses the curve's angle unit; TA curves expect radians
        unsigned int idx;
        curveFn.addKey(time, vals[i], MFnAnimCurve::kTangentAuto,
                       MFnAnimCurve::kTangentAuto, nullptr, &st);
    }
}

// ============================================================
// writeTranslationKey — set one (X,Y,Z) translation keyframe
// ============================================================
static void writeTranslationKey(const MDagPath&  joint,
                                const MPoint&    pos,
                                const MTime&     time)
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

// ============================================================
// writeAllKeys — write keyframes for all joints at one frame
// ============================================================
static void writeAllKeys(const BFSkeleton&       skel,
                         const BFManeuverAngles&  ang,
                         const MTime&             time)
{
    // Thorax
    writeRotationKey(skel.joints[kThorax],
        MEulerRotation(deg2rad(ang.thetaBeta), 0.0, 0.0), time);

    // Forewing L
    writeRotationKey(skel.joints[kForewingL],
        MEulerRotation(deg2rad(ang.thetaZeta),
                       deg2rad(ang.thetaPsi),
                       deg2rad(ang.thetaGamma)), time);

    // Forewing R (same angles — mirrorBehavior joints handle mirroring)
    writeRotationKey(skel.joints[kForewingR],
        MEulerRotation(deg2rad(ang.thetaZeta),
                       deg2rad(ang.thetaPsi),
                       deg2rad(ang.thetaGamma)), time);

    // Hindwing L
    writeRotationKey(skel.joints[kHindwingL],
        MEulerRotation(0.0, 0.0, deg2rad(ang.thetaGamma)), time);

    // Hindwing R (same angle — mirrorBehavior)
    writeRotationKey(skel.joints[kHindwingR],
        MEulerRotation(0.0, 0.0, deg2rad(ang.thetaGamma)), time);

    // Abdomen
    writeRotationKey(skel.joints[kAbdomen],
        MEulerRotation(deg2rad(ang.thetaPhi), 0.0, 0.0), time);
}

// ============================================================
// doIt — entry point when the MEL layer calls "bfSimulate"
// ============================================================
MStatus BFSimulateCmd::doIt(const MArgList& args)
{
    MStatus status;
    MArgDatabase argData(syntax(), args, &status);
    if (status != MS::kSuccess) {
        MGlobal::displayError("ButterFlight: Failed to parse command flags.");
        return status;
    } 

    // ---- Read the rig root joint name ------------------------
    MString rigName;
    if (argData.isFlagSet(kRigFlag)) {
        argData.getFlagArgument(kRigFlag, 0, rigName);
    } else {
        MGlobal::displayError("ButterFlight: -rig flag is required.");
        return MS::kInvalidParameter;
    }
     
    // ---- Resolve skeleton ------------------------------------
    status = readSkeleton(rigName, m_state.skeleton);
    if (status != MS::kSuccess) return status;

    // ---- Parse optional simulation flags -----------------------
    int duration   = 60;
    int startFrame = 1;
    double flapPeriod = 1.0;

    if (argData.isFlagSet(kDurFlag))
        argData.getFlagArgument(kDurFlag, 0, duration);
    if (argData.isFlagSet(kStartFlag))
        argData.getFlagArgument(kStartFlag, 0, startFrame);
    if (argData.isFlagSet(kFlapPeriodFlag))
        argData.getFlagArgument(kFlapPeriodFlag, 0, flapPeriod);

    if (flapPeriod <= 0.0) flapPeriod = 1.0;

    // Read FPS directly from Maya's scene time unit so that baked
    // keyframes always match the playback rate.  This prevents the
    // mismatch where a user-supplied FPS differs from Maya's setting
    // and inadvertently rescales flap speed.
    double fps = MTime(1.0, MTime::kSeconds).as(MTime::uiUnit());
    if (fps <= 0.0) fps = 24.0;

    // Convert artist-facing flapPeriod to engine simRate.
    // f_gamma_default is the initial gamma frequency (species constant:
    // 5.5 Hz for Monarch, from Table 3 of Chen et al. 2022).
    // simRate = simulation seconds per playback second (analogous to
    // Unity's Time.timeScale).
    double f_gamma_default = m_state.perAngleFreq[kAngleGamma];  // 5.5 Hz
    double simRate = 1.0 / (f_gamma_default * flapPeriod);
    double dt_sim  = simRate / fps;

    // ---- Initialise wing model (Monarch defaults) ------------
    BFWingModel wingModel;

    // ---- Simulation loop (pure kinematics, no forces) --------
    //  Uses constant default freq/amp from BFState to loop the
    //  same flapping motion every cycle.
    //  dt_sim decouples simulation time from FPS: changing FPS only
    //  changes keyframe density, not visible flap speed.
    for (int f = startFrame; f < startFrame + duration; ++f) {

        // 1. Advance phase and evaluate angles.
        //    No cycle-boundary detection needed — cos() is naturally
        //    periodic, so constant freq/amp produce identical motion
        //    every cycle with no timing seams.
        m_state.phase += dt_sim;
        wingModel.updateAnglesOnly(m_state);

        // 2. Apply joint rotations
        applyAngles(m_state.skeleton, m_state.angles);

        // 3. Write keyframes
        MTime frameTime((double)f, MTime::uiUnit());
        writeAllKeys(m_state.skeleton, m_state.angles, frameTime);
    }

    // ---- Set playback range to cover baked frames ---------------
    MAnimControl::setMinTime(MTime((double)startFrame, MTime::uiUnit()));
    MAnimControl::setMaxTime(MTime((double)(startFrame + duration - 1), MTime::uiUnit()));

    // ---- Report -----------------------------------------------
    MGlobal::displayInfo(
        MString("ButterFlight: Baked ") + duration +
        " frames (" + startFrame + "-" + (startFrame + duration - 1) +
        "), " + m_state.flapCycle + " flap cycles.");
    return MS::kSuccess;
}

// ============================================================
// undoIt / redoIt stubs
// ============================================================
MStatus BFSimulateCmd::undoIt()
{
    // TODO: restore cached animation curves
    return MS::kSuccess;
}

MStatus BFSimulateCmd::redoIt()
{
    // TODO: re-apply simulation keyframes
    return MS::kSuccess;
}
