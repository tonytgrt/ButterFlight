#pragma once
// ============================================================
// BFTestAeroCmd.h
// ButterFlight — Test command for aerodynamics validation
// ============================================================

#include <maya/MPxCommand.h>
#include <maya/MArgList.h>
#include <maya/MStatus.h>

class BFTestAeroCmd : public MPxCommand
{
public:
    BFTestAeroCmd() = default;
    ~BFTestAeroCmd() override = default;

    MStatus doIt(const MArgList& args) override;
    bool    isUndoable() const override { return false; }

    static void* creator() { return new BFTestAeroCmd; }

    static const char* kCommandName;
};
