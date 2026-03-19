#pragma once
// ============================================================
// BFTestIntegrationCmd.h
// ButterFlight — Integration test: WingModel + Aerodynamics
// ============================================================

#include <maya/MPxCommand.h>
#include <maya/MArgList.h>
#include <maya/MStatus.h>

class BFTestIntegrationCmd : public MPxCommand
{
public:
    BFTestIntegrationCmd() = default;
    ~BFTestIntegrationCmd() override = default;

    MStatus doIt(const MArgList& args) override;
    bool    isUndoable() const override { return false; }

    static void* creator() { return new BFTestIntegrationCmd; }

    static const char* kCommandName;
};
