#pragma once
// ============================================================
// BFSimulateCmd.h
// ButterFlight — Main simulation command (MPxCommand)
// ============================================================

#include <maya/MPxCommand.h>
#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>
#include <maya/MString.h>
#include <maya/MStatus.h>

#include "BFState.h"

class BFSimulateCmd : public MPxCommand
{
public:
    BFSimulateCmd() = default;
    ~BFSimulateCmd() override = default;

    // MPxCommand interface
    MStatus doIt(const MArgList& args) override;
    MStatus undoIt() override;
    MStatus redoIt() override;
    bool    isUndoable() const override { return true; }

    static void*   creator()  { return new BFSimulateCmd; }
    static MSyntax newSyntax();

    static const char* kCommandName;

    // ----------------------------------------------------------
    // Skeleton I/O
    // ----------------------------------------------------------

    /// Walk the joint hierarchy under \p rootJointName and populate
    /// \p outSkeleton with the DAG paths for every expected BF_ joint.
    /// Returns MS::kSuccess when all six joints are found.
    static MStatus readSkeleton(const MString& rootJointName,
                                BFSkeleton&    outSkeleton);

private:
    // Simulation state for the current run (supports undo)
    BFState m_state;
};
