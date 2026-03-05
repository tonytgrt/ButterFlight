// ============================================================
// BFSimulateCmd.cpp
// ButterFlight — Main simulation command implementation
// ============================================================

#include "BFSimulateCmd.h"

#include <maya/MGlobal.h>
#include <maya/MSelectionList.h>
#include <maya/MDagPath.h>
#include <maya/MFnDagNode.h>
#include <maya/MItDag.h>
#include <maya/MFn.h>
#include <maya/MArgList.h>

// ---- Command name registered with Maya ---------------------
const char* BFSimulateCmd::kCommandName = "bfSimulate";

// ---- Flag constants ----------------------------------------
static const char* kRigFlag      = "-r";
static const char* kRigFlagLong  = "-rig";
static const char* kModeFlag     = "-m";
static const char* kModeFlagLong = "-mode";

// ============================================================
// newSyntax — declare accepted flags
// ============================================================
MSyntax BFSimulateCmd::newSyntax()
{
    MSyntax syntax;
    syntax.addFlag(kRigFlag,  kRigFlagLong,  MSyntax::kString);
    syntax.addFlag(kModeFlag, kModeFlagLong, MSyntax::kLong);
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

    // ---- 2. Build a name-to-index map for expected children --
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

    // ---- 3. Depth-first traversal of the hierarchy -----------
    //  MItDag iterates every DAG node under rootPath that passes
    //  the MFn::kJoint filter, so we visit only joint nodes.
    MItDag dagIter(MItDag::kDepthFirst, MFn::kJoint, &status);
    if (status != MS::kSuccess) {
        MGlobal::displayError(
            "ButterFlight: Failed to create DAG iterator.");
        return MS::kFailure;
    }
    status = dagIter.reset(rootPath, MItDag::kDepthFirst, MFn::kJoint);
    if (status != MS::kSuccess) {
        MGlobal::displayError(
            "ButterFlight: Failed to reset DAG iterator to root.");
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
// doIt — entry point when the MEL layer calls "bfSimulate"
// ============================================================
MStatus BFSimulateCmd::doIt(const MArgList& args)
{
    MStatus status;
    MArgDatabase argData(newSyntax(), args, &status);
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

    // TODO: parse remaining flags, run simulation loop, write keyframes
    MGlobal::displayInfo("ButterFlight: Skeleton ready. Simulation not yet implemented.");
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
