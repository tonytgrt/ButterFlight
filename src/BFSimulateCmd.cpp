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
#include <maya/MFnCamera.h>
#include <maya/MMatrix.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MQuaternion.h>
#include <maya/MVector.h>
#include <maya/MStringArray.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static inline double deg2rad(double d) { return d * M_PI / 180.0; }

// ---- Unit conversion (Maya cm ↔ physics SI metres) ----------
static constexpr double kCmToM = 0.01;
static constexpr double kMToCm = 100.0;

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
static const char* kPathFlag           = "-p";
static const char* kPathFlagLong       = "-path";
static const char* kPathSpeedFlag      = "-ps";
static const char* kPathSpeedFlagLong  = "-pathSpeed";
static const char* kPathNoiseFlag      = "-pn";
static const char* kPathNoiseFlagLong  = "-pathNoise";
static const char* kPathNoiseAmpFlag      = "-pna";
static const char* kPathNoiseAmpFlagLong  = "-pathNoiseAmp";
static const char* kHoverPosXFlag      = "-hpx";
static const char* kHoverPosXFlagLong  = "-hoverPosX";
static const char* kHoverPosYFlag      = "-hpy";
static const char* kHoverPosYFlagLong  = "-hoverPosY";
static const char* kHoverPosZFlag      = "-hpz";
static const char* kHoverPosZFlagLong  = "-hoverPosZ";
static const char* kHoverRotXFlag      = "-hrx";
static const char* kHoverRotXFlagLong  = "-hoverRotX";
static const char* kHoverRotYFlag      = "-hry";
static const char* kHoverRotYFlagLong  = "-hoverRotY";
static const char* kHoverRotZFlag      = "-hrz";
static const char* kHoverRotZFlagLong  = "-hoverRotZ";
// Follow-camera flags
static const char* kCamFlag            = "-cam";
static const char* kCamFlagLong        = "-createCamera";
static const char* kCamOffXFlag        = "-cox";
static const char* kCamOffXFlagLong    = "-camOffsetX";
static const char* kCamOffYFlag        = "-coy";
static const char* kCamOffYFlagLong    = "-camOffsetY";
static const char* kCamOffZFlag        = "-coz";
static const char* kCamOffZFlagLong    = "-camOffsetZ";
static const char* kCamRotXFlag        = "-crx";
static const char* kCamRotXFlagLong    = "-camRotX";
static const char* kCamRotYFlag        = "-cry";
static const char* kCamRotYFlagLong    = "-camRotY";
static const char* kCamRotZFlag        = "-crz";
static const char* kCamRotZFlagLong    = "-camRotZ";
static const char* kCamStiffFlag       = "-cst";
static const char* kCamStiffFlagLong   = "-camStiffness";
static const char* kCamFOVFlag         = "-cfv";
static const char* kCamFOVFlagLong     = "-camFOV";
static const char* kCamNameFlag        = "-cnm";
static const char* kCamNameFlagLong    = "-camName";

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
    syntax.addFlag(kPathFlag, kPathFlagLong, MSyntax::kString);
    syntax.addFlag(kPathSpeedFlag, kPathSpeedFlagLong, MSyntax::kDouble);
    syntax.addFlag(kPathNoiseFlag, kPathNoiseFlagLong, MSyntax::kBoolean);
    syntax.addFlag(kPathNoiseAmpFlag, kPathNoiseAmpFlagLong, MSyntax::kDouble);
    syntax.addFlag(kHoverPosXFlag, kHoverPosXFlagLong, MSyntax::kDouble);
    syntax.addFlag(kHoverPosYFlag, kHoverPosYFlagLong, MSyntax::kDouble);
    syntax.addFlag(kHoverPosZFlag, kHoverPosZFlagLong, MSyntax::kDouble);
    syntax.addFlag(kHoverRotXFlag, kHoverRotXFlagLong, MSyntax::kDouble);
    syntax.addFlag(kHoverRotYFlag, kHoverRotYFlagLong, MSyntax::kDouble);
    syntax.addFlag(kHoverRotZFlag, kHoverRotZFlagLong, MSyntax::kDouble);
    syntax.addFlag(kCamFlag,      kCamFlagLong,      MSyntax::kBoolean);
    syntax.addFlag(kCamOffXFlag,  kCamOffXFlagLong,  MSyntax::kDouble);
    syntax.addFlag(kCamOffYFlag,  kCamOffYFlagLong,  MSyntax::kDouble);
    syntax.addFlag(kCamOffZFlag,  kCamOffZFlagLong,  MSyntax::kDouble);
    syntax.addFlag(kCamRotXFlag,  kCamRotXFlagLong,  MSyntax::kDouble);
    syntax.addFlag(kCamRotYFlag,  kCamRotYFlagLong,  MSyntax::kDouble);
    syntax.addFlag(kCamRotZFlag,  kCamRotZFlagLong,  MSyntax::kDouble);
    syntax.addFlag(kCamStiffFlag, kCamStiffFlagLong, MSyntax::kDouble);
    syntax.addFlag(kCamFOVFlag,   kCamFOVFlagLong,   MSyntax::kDouble);
    syntax.addFlag(kCamNameFlag,  kCamNameFlagLong,  MSyntax::kString);
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
                        const BFManeuverAngles&  ang,
                        double                   heading)
{
    MStatus st;

    // Thorax pitch + heading (yaw)
    MFnTransform thoraxFn(skel.joints[kThorax], &st);
    if (st == MS::kSuccess) {
        thoraxFn.setRotation(MEulerRotation(
            deg2rad(ang.thetaBeta), heading, 0.0,
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
// clearAnimCurves — remove all existing keys from a joint's
// rotation and translation anim curves so that re-simulation
// doesn't leave stale keys from a previous run.
// ============================================================
static void clearAnimCurves(const MDagPath& joint)
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

        // Remove all keys (iterate in reverse to keep indices valid)
        for (int i = (int)curveFn.numKeys() - 1; i >= 0; --i)
            curveFn.remove(i);
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

        // addKey uses the curve's angle unit; TA curves expect radians.
        // Linear tangents prevent overshoot between per-frame keys.
        unsigned int idx;
        curveFn.addKey(time, vals[i], MFnAnimCurve::kTangentLinear,
                       MFnAnimCurve::kTangentLinear, nullptr, &st);
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

        // Linear tangents prevent overshoot between per-frame keys,
        // keeping the butterfly's speed even along the path.
        curveFn.addKey(time, vals[i], MFnAnimCurve::kTangentLinear,
                       MFnAnimCurve::kTangentLinear, nullptr, &st);
    }
}

// ============================================================
// writeAllKeys — write keyframes for all joints at one frame
// ============================================================
static void writeAllKeys(const BFSkeleton&       skel,
                         const BFManeuverAngles&  ang,
                         double                   heading,
                         const MTime&             time)
{
    // Thorax — pitch + heading (yaw)
    writeRotationKey(skel.joints[kThorax],
        MEulerRotation(deg2rad(ang.thetaBeta), heading, 0.0), time);

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
// bakeFollowCamera — create/reuse a spring-arm follow camera and
// bake per-frame translation+rotation keys from the recorded
// butterfly positions and headings.
//
//   positions[i]     : world-space butterfly thorax position in
//                      Maya cm, one per output frame
//   headings[i]      : butterfly yaw in radians (heading=0 faces -Z)
//   startFrame       : frame number of positions[0]
//   camName          : transform node name (reused across re-runs)
//   localOffsetCm    : camera offset in butterfly's local frame
//                      (X right, Y up, Z behind)
//   rotOffset        : additional Euler rotation applied in camera
//                      local frame on top of the look-at orientation
//   stiffnessPerSec  : spring follow rate in 1/seconds
//   fovDeg           : camera vertical FOV in degrees
//   fps              : output frame rate (for stiffness->alpha conv.)
// ============================================================
static void bakeFollowCamera(
    const std::vector<MPoint>&  positions,
    const std::vector<double>&  headings,
    int                         startFrame,
    const MString&              camName,
    MVector                     localOffsetCm,
    MEulerRotation              rotOffset,
    double                      stiffnessPerSec,
    double                      fovDeg,
    double                      fps)
{
    if (positions.empty() || positions.size() != headings.size()) {
        MGlobal::displayWarning(
            "ButterFlight: camera bake skipped (no recorded frames).");
        return;
    }
    if (fps <= 0.0)             fps = 24.0;
    if (stiffnessPerSec <= 0.0) stiffnessPerSec = 10.0;
    if (fovDeg < 5.0)           fovDeg = 5.0;
    if (fovDeg > 170.0)         fovDeg = 170.0;

    MStatus st;

    // ---- Find or create camera transform -----------------------
    //  Use the MEL `camera -name` command for creation — it creates
    //  the transform+shape pair atomically with the requested name.
    //  The MFnCamera::create API left the transform named "camera1"
    //  on some Maya versions (rename via MFnDagNode::setName was
    //  inconsistent), so we avoid it entirely.
    MDagPath camTransformPath;
    bool reused = false;
    {
        MSelectionList sel;
        if (sel.add(camName) == MS::kSuccess && sel.length() > 0) {
            MDagPath tpath;
            if (sel.getDagPath(0, tpath) == MS::kSuccess
                && tpath.hasFn(MFn::kTransform)) {
                MFnDagNode dagFn(tpath);
                for (unsigned int i = 0; i < dagFn.childCount(); ++i) {
                    MObject child = dagFn.child(i);
                    if (child.hasFn(MFn::kCamera)) {
                        camTransformPath = tpath;
                        reused = true;
                        MFnCamera camFn(child, &st);
                        if (st == MS::kSuccess)
                            camFn.setVerticalFieldOfView(deg2rad(fovDeg));
                        break;
                    }
                }
                if (!reused) {
                    MGlobal::displayWarning(
                        "ButterFlight: node '" + camName +
                        "' exists but has no camera shape — will create with a suffix.");
                }
            }
        }
    }
    if (!reused) {
        MStringArray createResult;
        MString createCmd = MString("camera -name \"") + camName + "\"";
        MStatus cmdSt = MGlobal::executeCommand(createCmd, createResult);
        if (cmdSt != MS::kSuccess || createResult.length() < 1) {
            MGlobal::displayError(
                "ButterFlight: Failed to create follow camera via MEL.");
            return;
        }
        // createResult[0] is the transform's short name (may be
        // suffixed, e.g. "BF_followCam1" if the target was taken).
        MString actualName = createResult[0];
        MSelectionList sel2;
        if (sel2.add(actualName) != MS::kSuccess || sel2.length() == 0) {
            MGlobal::displayError(
                "ButterFlight: Camera '" + actualName +
                "' was created but could not be resolved.");
            return;
        }
        if (sel2.getDagPath(0, camTransformPath) != MS::kSuccess
            || !camTransformPath.hasFn(MFn::kTransform)) {
            MGlobal::displayError(
                "ButterFlight: Camera DAG path resolution failed.");
            return;
        }
        // Set FOV on the shape child.
        for (unsigned int i = 0; i < camTransformPath.childCount(); ++i) {
            MObject child = camTransformPath.child(i);
            if (child.hasFn(MFn::kCamera)) {
                MFnCamera camFn(child, &st);
                if (st == MS::kSuccess)
                    camFn.setVerticalFieldOfView(deg2rad(fovDeg));
                break;
            }
        }
        MGlobal::displayInfo(
            MString("ButterFlight: Created follow camera '") +
            actualName + "' (requested '" + camName + "').");
    } else {
        MGlobal::displayInfo(
            MString("ButterFlight: Reusing follow camera '") +
            camName + "'.");
    }

    // Sanity check: the path must be valid before we key it.
    if (!camTransformPath.isValid() || camTransformPath.node().isNull()) {
        MGlobal::displayError(
            "ButterFlight: Camera transform path is invalid — aborting bake.");
        return;
    }

    // ---- Clear any previous keys on the transform ---------------
    clearAnimCurves(camTransformPath);

    // ---- Per-frame bake ----------------------------------------
    const int nFrames = (int)positions.size();
    const double alpha = 1.0 - std::exp(-stiffnessPerSec / fps);

    // Frame-0 seed: camera starts exactly at its rest position so
    // there is no initial snap.
    MPoint prevCamPos;

    // Heading unwrap so that a ±pi discontinuity in the stored yaw
    // doesn't cause the offset to spin the long way around.
    double unwrapped = headings[0];

    for (int f = 0; f < nFrames; ++f) {
        double raw = headings[f];
        while (raw - unwrapped >  M_PI) raw -= 2.0 * M_PI;
        while (raw - unwrapped < -M_PI) raw += 2.0 * M_PI;
        unwrapped = raw;

        // Rotate localOffsetCm around world Y by the butterfly's yaw.
        // heading = 0 means facing -Z, so the rotation that sends
        // local +Z to the world "behind butterfly" direction is:
        //   x' =  cos(h) * x + sin(h) * z
        //   z' = -sin(h) * x + cos(h) * z
        const double cosY = std::cos(unwrapped);
        const double sinY = std::sin(unwrapped);
        const MVector offsetWorld(
            cosY * localOffsetCm.x + sinY * localOffsetCm.z,
            localOffsetCm.y,
            -sinY * localOffsetCm.x + cosY * localOffsetCm.z);
        const MPoint pDesired = positions[f] + offsetWorld;

        MPoint camPos;
        if (f == 0) {
            camPos = pDesired;  // seed with no lag
        } else {
            camPos = prevCamPos + (pDesired - prevCamPos) * alpha;
        }

        // ---- Look-at rotation ---------------------------------
        // Maya cameras look down -Z with +Y up and +X right.
        MEulerRotation finalRot(0.0, 0.0, 0.0);
        MVector forward = MVector(positions[f]) - MVector(camPos);
        const double fwdLen = forward.length();
        if (fwdLen < 1e-6) {
            finalRot = rotOffset;
        } else {
            forward /= fwdLen;
            MVector up(0.0, 1.0, 0.0);
            // Degenerate when looking straight up/down: swap to +Z up.
            if (std::abs(forward * up) > 0.999) {
                up = MVector(0.0, 0.0, 1.0);
            }
            MVector camZ = -forward;     // camera +Z points away from target
            MVector camX = up ^ camZ;    // right
            if (camX.length() < 1e-6) camX = MVector(1.0, 0.0, 0.0);
            camX.normalize();
            MVector camY = camZ ^ camX;  // up'
            camY.normalize();

            // Build a 4x4 whose rows are the camera basis in world
            // (Maya uses row vectors, so v_world = v_local * M).
            double m[4][4] = {
                { camX.x, camX.y, camX.z, 0.0 },
                { camY.x, camY.y, camY.z, 0.0 },
                { camZ.x, camZ.y, camZ.z, 0.0 },
                { 0.0,    0.0,    0.0,    1.0 }
            };
            MMatrix mat(m);
            MTransformationMatrix xform(mat);
            MQuaternion baseQ  = xform.rotation();
            MQuaternion offQ   = rotOffset.asQuaternion();
            // q_final = offset * base applies offset in the camera's
            // local frame first, then the look-at orientation.
            MQuaternion finalQ = offQ * baseQ;
            finalRot = finalQ.asEulerRotation();
        }

        const MTime t((double)(startFrame + f), MTime::uiUnit());
        writeTranslationKey(camTransformPath, camPos, t);
        writeRotationKey   (camTransformPath, finalRot, t);

        prevCamPos = camPos;
    }

    // Diagnostic: first/last sample so we can confirm motion was baked.
    const MPoint& firstBf  = positions.front();
    const MPoint& lastBf   = positions.back();
    MGlobal::displayInfo(
        MString("ButterFlight: Follow camera '") + camTransformPath.partialPathName() +
        "' baked " + nFrames + " frames (stiffness=" + stiffnessPerSec +
        ", fov=" + fovDeg + " deg).");
    MGlobal::displayInfo(
        MString("  butterfly cm  : first=(") +
        firstBf.x + ", " + firstBf.y + ", " + firstBf.z +
        ")  last=(" + lastBf.x + ", " + lastBf.y + ", " + lastBf.z + ")");
    MGlobal::displayInfo(
        MString("  local offset  : (") +
        localOffsetCm.x + ", " + localOffsetCm.y + ", " + localOffsetCm.z + ") cm");
    MGlobal::displayInfo(
        MString("  prevCamPos(end)=(") +
        prevCamPos.x + ", " + prevCamPos.y + ", " + prevCamPos.z + ")");
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

    double pathSpeedScale = 1.0;
    if (argData.isFlagSet(kPathSpeedFlag))
        argData.getFlagArgument(kPathSpeedFlag, 0, pathSpeedScale);
    if (pathSpeedScale <= 0.0) pathSpeedScale = 1.0;

    // Path noise: lateral wandering around the curve so the butterfly
    // doesn't track the path with mechanical precision.
    bool   pathNoiseEnable = false;
    double pathNoiseAmpCm  = 5.0;   // lateral amplitude in Maya cm
    if (argData.isFlagSet(kPathNoiseFlag))
        argData.getFlagArgument(kPathNoiseFlag, 0, pathNoiseEnable);
    if (argData.isFlagSet(kPathNoiseAmpFlag))
        argData.getFlagArgument(kPathNoiseAmpFlag, 0, pathNoiseAmpCm);
    if (pathNoiseAmpCm < 0.0) pathNoiseAmpCm = 0.0;
    double pathNoiseAmpM = pathNoiseAmpCm * kCmToM;

    // ---- Detect hover mode (mode == 4) ----------------------------
    bool hoverMode = false;
    bool hoverHasCustomPos = false;
    double hoverPosXcm = 0.0, hoverPosYcm = 0.0, hoverPosZcm = 0.0;
    double hoverRotXdeg = 0.0, hoverRotYdeg = 0.0, hoverRotZdeg = 0.0;
    if (argData.isFlagSet(kModeFlag)) {
        int mode = 1;
        argData.getFlagArgument(kModeFlag, 0, mode);
        hoverMode = (mode == 4);
    }
    if (argData.isFlagSet(kHoverPosXFlag) || argData.isFlagSet(kHoverPosYFlag)
        || argData.isFlagSet(kHoverPosZFlag)) {
        hoverHasCustomPos = true;
    }
    bool hoverHasCustomRot = false;
    if (argData.isFlagSet(kHoverRotXFlag) || argData.isFlagSet(kHoverRotYFlag)
        || argData.isFlagSet(kHoverRotZFlag)) {
        hoverHasCustomRot = true;
    }
    if (argData.isFlagSet(kHoverPosXFlag))
        argData.getFlagArgument(kHoverPosXFlag, 0, hoverPosXcm);
    if (argData.isFlagSet(kHoverPosYFlag))
        argData.getFlagArgument(kHoverPosYFlag, 0, hoverPosYcm);
    if (argData.isFlagSet(kHoverPosZFlag))
        argData.getFlagArgument(kHoverPosZFlag, 0, hoverPosZcm);
    if (argData.isFlagSet(kHoverRotXFlag))
        argData.getFlagArgument(kHoverRotXFlag, 0, hoverRotXdeg);
    if (argData.isFlagSet(kHoverRotYFlag))
        argData.getFlagArgument(kHoverRotYFlag, 0, hoverRotYdeg);
    if (argData.isFlagSet(kHoverRotZFlag))
        argData.getFlagArgument(kHoverRotZFlag, 0, hoverRotZdeg);

    // ---- Follow-camera flags ---------------------------------------
    bool   camEnable    = false;
    MString camName     = "BF_followCam";
    double camOffXcm    = 0.0;
    double camOffYcm    = 5.0;
    double camOffZcm    = 30.0;
    double camRotXdeg   = 0.0;
    double camRotYdeg   = 0.0;
    double camRotZdeg   = 0.0;
    double camStiffness = 10.0;
    double camFOVdeg    = 35.0;
    if (argData.isFlagSet(kCamFlag))
        argData.getFlagArgument(kCamFlag, 0, camEnable);
    if (argData.isFlagSet(kCamNameFlag))
        argData.getFlagArgument(kCamNameFlag, 0, camName);
    if (argData.isFlagSet(kCamOffXFlag))
        argData.getFlagArgument(kCamOffXFlag, 0, camOffXcm);
    if (argData.isFlagSet(kCamOffYFlag))
        argData.getFlagArgument(kCamOffYFlag, 0, camOffYcm);
    if (argData.isFlagSet(kCamOffZFlag))
        argData.getFlagArgument(kCamOffZFlag, 0, camOffZcm);
    if (argData.isFlagSet(kCamRotXFlag))
        argData.getFlagArgument(kCamRotXFlag, 0, camRotXdeg);
    if (argData.isFlagSet(kCamRotYFlag))
        argData.getFlagArgument(kCamRotYFlag, 0, camRotYdeg);
    if (argData.isFlagSet(kCamRotZFlag))
        argData.getFlagArgument(kCamRotZFlag, 0, camRotZdeg);
    if (argData.isFlagSet(kCamStiffFlag))
        argData.getFlagArgument(kCamStiffFlag, 0, camStiffness);
    if (argData.isFlagSet(kCamFOVFlag))
        argData.getFlagArgument(kCamFOVFlag, 0, camFOVdeg);

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

    // ---- Decouple physics rate from output fps ----------------
    //  The -frameRate flag sets the PLAYBACK / KEYFRAME rate, not the
    //  physics timestep.  Physics runs at a fixed ~960 Hz regardless
    //  of output fps, so:
    //    • wing flap speed is independent of the fps flag
    //    • maneuvering forces integrate at consistent precision
    //    • velocity/position don't accumulate over-large steps at
    //      low output fps (which was causing the amplitude discontinuity)
    static constexpr double kTargetSimHz = 960.0;
    int    substeps = std::max(1, (int)std::ceil(kTargetSimHz / fps));
    double simDt = simRate / (fps * substeps);  // physics timestep (≈1/960s)

    // ---- Initialise wing model and maneuvering controller ----
    BFWingModel wingModel;
    BFManeuverController controller;
    controller.maxSpeed = wingModel.maxSpeed;

    // ---- Resolve path curve (optional) --------------------------
    MDagPath curveDagPath;
    MFnNurbsCurve curveFn;
    bool hasPath = false;

    if (argData.isFlagSet(kPathFlag)) {
        MString curveName;
        argData.getFlagArgument(kPathFlag, 0, curveName);
        MSelectionList sel;
        sel.add(curveName);
        sel.getDagPath(0, curveDagPath);
        // If user selected transform, extend to shape
        if (curveDagPath.hasFn(MFn::kTransform))
            curveDagPath.extendToShape();
        if (curveDagPath.hasFn(MFn::kNurbsCurve)) {
            curveFn.setObject(curveDagPath);
            hasPath = true;
            MGlobal::displayInfo("ButterFlight: Path curve '" + curveName + "' loaded.");
        } else {
            MGlobal::displayWarning("ButterFlight: '" + curveName +
                "' is not a NURBS curve — ignoring path.");
        }
    }

    // Seed position and heading from the previous frame so that a
    // simulation starting at e.g. frame 961 picks up smoothly from
    // the animation already baked up to frame 960.
    if (startFrame > 1) {
        MTime prevTime((double)(startFrame - 1), MTime::uiUnit());
        MAnimControl::setCurrentTime(prevTime);
    }
    {
        MFnTransform rootFn(m_state.skeleton.joints[kThorax]);
        MVector t = rootFn.getTranslation(MSpace::kWorld);
        m_state.position = MPoint(t.x * kCmToM, t.y * kCmToM, t.z * kCmToM);

        // Read heading from the previous frame's Y rotation
        MEulerRotation rot;
        rootFn.getRotation(rot);
        m_state.heading = rot.y;
    }
    if (startFrame > 1) {
        // Seed a gentle forward velocity in the heading direction so
        // that free-flight's heading-from-velocity doesn't snap to an
        // arbitrary direction when continuing from hover (v ≈ 0).
        double initSpeed = 0.5;  // m/s — mild nudge forward
        m_state.velocity = MVector(-std::sin(m_state.heading) * initSpeed,
                                    0.0,
                                   -std::cos(m_state.heading) * initSpeed);
        MGlobal::displayInfo(
            MString("ButterFlight: Seeded from frame ") + (startFrame - 1) +
            " — pos=(" + m_state.position.x * kMToCm + ", " +
            m_state.position.y * kMToCm + ", " +
            m_state.position.z * kMToCm + ") cm, heading=" +
            (m_state.heading * 180.0 / M_PI) + " deg");
    }

    // If a path is provided and this is a fresh start, snap to curve start
    if (hasPath && startFrame <= 1) {
        double uMin, uMax;
        curveFn.getKnotDomain(uMin, uMax);
        MPoint startPt;  // Maya cm
        curveFn.getPointAtParam(uMin, startPt, MSpace::kWorld);
        m_state.position = MPoint(startPt.x * kCmToM, startPt.y * kCmToM, startPt.z * kCmToM);

        MFnTransform rootFn(m_state.skeleton.joints[kThorax]);
        rootFn.setTranslation(MVector(startPt), MSpace::kWorld);  // stays in cm for Maya

        // Initialize heading to face along the curve's start tangent
        MVector tangent = curveFn.tangent(uMin, MSpace::kWorld);
        double tx = tangent.x, tz = tangent.z;
        if (std::sqrt(tx * tx + tz * tz) > 1e-6) {
            m_state.heading = std::atan2(-tx, -tz);
        }
    }

    // ---- Hover mode: fix position and orientation -------------------
    //  hoverPitchRad / hoverRollRad are base orientation offsets (radians).
    //  They are added on top of the wing model's thetaBeta oscillation
    //  in the simulation loop so the body bob is preserved.
    double hoverPitchRad = 0.0;
    double hoverRollRad  = 0.0;
    if (hoverMode) {
        if (startFrame <= 1) {
            // Fresh start: apply hover position/rotation from flags
            if (hoverHasCustomPos) {
                m_state.position = MPoint(hoverPosXcm * kCmToM,
                                          hoverPosYcm * kCmToM,
                                          hoverPosZcm * kCmToM);
                MFnTransform rootFn(m_state.skeleton.joints[kThorax]);
                rootFn.setTranslation(MVector(hoverPosXcm, hoverPosYcm, hoverPosZcm),
                                      MSpace::kWorld);
            }
            if (hoverHasCustomRot) {
                m_state.heading  = deg2rad(hoverRotYdeg);
                hoverPitchRad    = deg2rad(hoverRotXdeg);
                hoverRollRad     = deg2rad(hoverRotZdeg);
            } else {
                MStatus rotSt;
                MFnTransform rootFn(m_state.skeleton.joints[kThorax], &rotSt);
                if (rotSt == MS::kSuccess) {
                    MEulerRotation rigRot;
                    rootFn.getRotation(rigRot);
                    hoverPitchRad   = rigRot.x;
                    m_state.heading = rigRot.y;
                    hoverRollRad    = rigRot.z;
                    hoverRotXdeg = rigRot.x * 180.0 / M_PI;
                    hoverRotYdeg = rigRot.y * 180.0 / M_PI;
                    hoverRotZdeg = rigRot.z * 180.0 / M_PI;
                }
            }
        } else {
            // Continuing from previous frame: read pitch/roll from seeded rotation
            MFnTransform rootFn(m_state.skeleton.joints[kThorax]);
            MEulerRotation rigRot;
            rootFn.getRotation(rigRot);
            hoverPitchRad = rigRot.x;
            hoverRollRad  = rigRot.z;
        }
        m_state.velocity = MVector(0.0, 0.0, 0.0);
        MGlobal::displayInfo(
            MString("ButterFlight: Hover mode at (") +
            m_state.position.x * kMToCm + ", " +
            m_state.position.y * kMToCm + ", " +
            m_state.position.z * kMToCm + ") cm, rot=(" +
            hoverRotXdeg + ", " + hoverRotYdeg + ", " + hoverRotZdeg + ") deg");
    }

    // ---- Path-progress tracking variables -------------------------
    bool   pathActive      = hasPath; // true while actively following
    double totalCurveLen   = 0.0;   // total curve length (cm)
    MPoint curveEndPtM;             // curve endpoint in metres
    if (hasPath) {
        totalCurveLen = curveFn.length();
        double uMin2, uMax2;
        curveFn.getKnotDomain(uMin2, uMax2);
        MPoint endCm;
        curveFn.getPointAtParam(uMax2, endCm, MSpace::kWorld);
        curveEndPtM = MPoint(endCm.x * kCmToM, endCm.y * kCmToM, endCm.z * kCmToM);
    }

    // ---- Compute path arc advancement rate --------------------------
    //  Instead of deriving a physics velocity (which gets clamped to
    //  maxSpeed and fights aero forces), advance a time-based cursor
    //  along the curve at a fixed rate per substep.  At scale 1.0 the
    //  cursor traverses the full curve over the full simulation.
    //  arcRate is in cm/substep (curve's native unit).
    double arcCursor = 0.0;
    double arcRate   = 0.0;
    double pathNoiseTime = 0.0;  // accumulated sim time for noise oscillator

    // Per-run random phase offsets so the wander pattern differs every
    // time the user re-simulates.  Seeded from std::random_device.
    double pnPhaseLatA = 0.0, pnPhaseLatB = 0.0;
    double pnPhaseVerA = 0.0, pnPhaseVerB = 0.0;
    if (pathNoiseEnable) {
        std::random_device rd;
        std::mt19937 rng(rd());
        std::uniform_real_distribution<double> dist(0.0, 2.0 * M_PI);
        pnPhaseLatA = dist(rng);
        pnPhaseLatB = dist(rng);
        pnPhaseVerA = dist(rng);
        pnPhaseVerB = dist(rng);
        MGlobal::displayInfo(
            MString("ButterFlight: path noise seed phases = ")
            + pnPhaseLatA + ", " + pnPhaseLatB + ", "
            + pnPhaseVerA + ", " + pnPhaseVerB);
    }
    if (hasPath && duration > 0 && substeps > 0) {
        arcRate = totalCurveLen / ((double)duration * substeps) * pathSpeedScale;
        MGlobal::displayInfo(
            MString("ButterFlight: arcRate=") + arcRate +
            " cm/substep (curveLen=" + totalCurveLen +
            " cm, scale=" + pathSpeedScale + ")");
    }

    MGlobal::displayInfo(
        MString("ButterFlight: simDt=") + simDt +
        "s (" + substeps + " substeps/frame @ " + fps + " fps output)");

    // ---- Follow-camera sample buffers ------------------------------
    // Only populated when the user requested a follow camera; we
    // record one sample per output frame and bake after the sim loop.
    std::vector<MPoint>  camBfPositions;
    std::vector<double>  camBfHeadings;
    if (camEnable) {
        camBfPositions.reserve((size_t)std::max(1, duration));
        camBfHeadings .reserve((size_t)std::max(1, duration));
    }

    // ---- Simulation loop (full dynamics) ---------------------
    for (int f = startFrame; f < startFrame + duration; ++f) {

        // Run multiple physics substeps per output frame so that
        // wing kinematics and body dynamics are fps-independent.
        for (int s = 0; s < substeps; ++s) {
            int prevCycle = m_state.flapCycle;

            if (hoverMode) {
                // ---- Hover: wing flapping only, no physics --------
                //  Run the full wing model so freq/amp adapt toward
                //  the zero-speed floor values (kMinFreq/kMinAmp),
                //  producing a natural hovering flap.  Position and
                //  velocity stay fixed — no forces, no gravity.
                wingModel.update(m_state, simDt);
                applyAngles(m_state.skeleton, m_state.angles, m_state.heading);

                // Apply user pitch/roll offsets on top of the wing
                // model's thetaBeta oscillation.  applyAngles() sets
                // thorax to (thetaBeta, heading, 0); we override to
                // (thetaBeta + userPitch, heading, userRoll).
                if (hoverPitchRad != 0.0 || hoverRollRad != 0.0) {
                    MStatus st;
                    MFnTransform thoraxFn(m_state.skeleton.joints[kThorax], &st);
                    if (st == MS::kSuccess) {
                        thoraxFn.setRotation(MEulerRotation(
                            deg2rad(m_state.angles.thetaBeta) + hoverPitchRad,
                            m_state.heading,
                            hoverRollRad,
                            MEulerRotation::kXYZ));
                    }
                }
                // Position/velocity/heading unchanged.
            } else {
                // ---- Flight modes (free flight / path following) ---

                // 1. Advance phase and evaluate maneuvering angles.
                //    In path-following mode, use fixed frequencies (the
                //    initial defaults) so that flap rate stays constant
                //    regardless of flight speed.  In free flight the
                //    velocity-adaptive sigmoid drives freq/amp as usual.
                if (hasPath) {
                    // Fixed-rate phase advancement
                    for (int i = 0; i < BFState::kNumAngles; ++i)
                        m_state.perAnglePhase[i] += 2.0 * M_PI * m_state.perAngleFreq[i] * simDt;
                    m_state.phase += simDt;
                    double cyclePeriod = (m_state.frequency > 0.0)
                                         ? 1.0 / m_state.frequency : 1.0;
                    if (m_state.phase >= cyclePeriod) {
                        m_state.phase -= cyclePeriod;
                        m_state.flapCycle++;
                    }
                    wingModel.updateAnglesOnly(m_state);
                } else {
                    wingModel.update(m_state, simDt);
                }

                // 2. Apply rotations so aerodynamics reads current normals.
                applyAngles(m_state.skeleton, m_state.angles, m_state.heading);

                // 3. Integrate forces → velocity → position (Eqs. 4-11).
                //    In path mode, skip controller.step() entirely so
                //    aero/vortex/gravity forces don't fight the path
                //    spring.  The cursor + spring have full control.
                if (!pathActive)
                    controller.step(m_state, simDt);

                // 3b. Path-following: time-based cursor steering.
                if (pathActive) {
                    // Advance cursor (time-based, not position-based)
                    arcCursor += arcRate;
                    if (arcCursor > totalCurveLen) arcCursor = totalCurveLen;

                    double uTarget = curveFn.findParamFromLength(arcCursor);

                    // Cursor point on curve (metres)
                    MPoint targetCm;
                    curveFn.getPointAtParam(uTarget, targetCm, MSpace::kWorld);
                    MVector targetM(targetCm.x * kCmToM,
                                    targetCm.y * kCmToM,
                                    targetCm.z * kCmToM);

                    // Curve tangent at cursor
                    MVector tangent = curveFn.tangent(uTarget, MSpace::kWorld);
                    tangent.normalize();

                    // ---- Optional path noise: lateral swing -------
                    // Adds a smooth left/right (and slight up/down)
                    // wander to the target point so the butterfly
                    // doesn't track the curve mechanically.  Uses a
                    // sum of two incommensurate sines per axis for
                    // an organic, non-repeating feel.
                    if (pathNoiseEnable && pathNoiseAmpM > 0.0) {
                        pathNoiseTime += simDt;
                        const double t = pathNoiseTime;
                        // Lateral axis: perpendicular to tangent in XZ
                        MVector up(0.0, 1.0, 0.0);
                        MVector lateral = tangent ^ up;
                        double lateralLen = lateral.length();
                        if (lateralLen > 1e-6) {
                            lateral /= lateralLen;
                            double sLat = std::sin(2.0 * M_PI * 0.7 * t + pnPhaseLatA)
                                        + 0.5 * std::sin(2.0 * M_PI * 1.13 * t + pnPhaseLatB);
                            double sVert = 0.4 * (std::sin(2.0 * M_PI * 0.5 * t + pnPhaseVerA)
                                                + 0.5 * std::sin(2.0 * M_PI * 0.91 * t + pnPhaseVerB));
                            targetM += lateral * (pathNoiseAmpM * sLat / 1.5);
                            targetM += up      * (pathNoiseAmpM * sVert / 1.5);
                        }
                    }

                    // Spring toward cursor point
                    MVector toTarget = targetM - MVector(m_state.position);
                    double dist = toTarget.length();

                    // Desired velocity: spring pull toward cursor.
                    // Gain of 15/s makes the butterfly converge quickly
                    // without overshooting.  No maxSpeed clamp — the
                    // cursor rate controls effective speed, not physics.
                    MVector desiredVel = toTarget * 15.0;
                    // Add tangent bias so direction stays smooth on curves
                    double tangentBias = arcRate * kCmToM / simDt * 0.3;
                    desiredVel += tangent * tangentBias;

                    // Blend toward desired (higher rate for tighter tracking)
                    double blendRate = 18.0;
                    double alpha = 1.0 - std::exp(-blendRate * simDt); 
                    m_state.velocity = m_state.velocity * (1.0 - alpha)
                                     + desiredVel * alpha;

                    // Heading from curve tangent
                    double tx = tangent.x, tz = tangent.z;
                    if (std::sqrt(tx * tx + tz * tz) > 1e-6)
                        m_state.heading = std::atan2(-tx, -tz);

                    // End-of-curve: transition to free flight
                    if (arcCursor >= totalCurveLen) {
                        double distToEnd = (MVector(curveEndPtM)
                                          - MVector(m_state.position)).length();
                        if (distToEnd < 0.1)
                            pathActive = false;
                    }
                }

                // 3c. Free-flight heading from velocity direction.
                if (!pathActive) {
                    double vx = m_state.velocity.x;
                    double vz = m_state.velocity.z;
                    double hSpeed = std::sqrt(vx * vx + vz * vz);
                    if (hSpeed > 0.01)
                        m_state.heading = std::atan2(-vx, -vz);
                }

                // 3d. In path mode, controller.step() was skipped, so
                //     integrate position here from the spring velocity.
                if (pathActive) {
                    m_state.position.x += m_state.velocity.x * simDt;
                    m_state.position.y += m_state.velocity.y * simDt;
                    m_state.position.z += m_state.velocity.z * simDt;
                }
            }

            // 4. Sliding-window smoother at each flap cycle boundary
            //    (Eq. 12).
            if (m_state.flapCycle != prevCycle)
                controller.smoothParameters(m_state);
        }

        // Write one keyframe per output frame (after all substeps).
        MTime frameTime((double)f, MTime::uiUnit());
        writeAllKeys(m_state.skeleton, m_state.angles, m_state.heading, frameTime);

        // In hover mode, overwrite the thorax rotation key to include
        // user pitch/roll offsets (writeAllKeys writes (thetaBeta, heading, 0)).
        if (hoverMode && (hoverPitchRad != 0.0 || hoverRollRad != 0.0)) {
            writeRotationKey(m_state.skeleton.joints[kThorax],
                MEulerRotation(
                    deg2rad(m_state.angles.thetaBeta) + hoverPitchRad,
                    m_state.heading,
                    hoverRollRad),
                frameTime);
        }
        MPoint posCm(m_state.position.x * kMToCm,
                     m_state.position.y * kMToCm,
                     m_state.position.z * kMToCm);
        writeTranslationKey(m_state.skeleton.joints[kThorax],
                            posCm, frameTime);

        // Record per-frame sample for the follow camera (cm + radians).
        if (camEnable) {
            camBfPositions.push_back(posCm);
            camBfHeadings .push_back(m_state.heading);
        }
    }

    // ---- Follow camera: bake keys from recorded samples --------
    if (camEnable && !camBfPositions.empty()) {
        bakeFollowCamera(
            camBfPositions,
            camBfHeadings,
            startFrame,
            camName,
            MVector(camOffXcm, camOffYcm, camOffZcm),
            MEulerRotation(deg2rad(camRotXdeg),
                           deg2rad(camRotYdeg),
                           deg2rad(camRotZdeg),
                           MEulerRotation::kXYZ),
            camStiffness,
            camFOVdeg,
            fps);
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
