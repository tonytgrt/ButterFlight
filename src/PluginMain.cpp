// ============================================================
// PluginMain.cpp
// ButterFlight — Maya plugin entry points
// ============================================================

#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>

#include "BFSimulateCmd.h"
#include "test/BFTestAeroCmd.h"
#include "test/BFTestIntegrationCmd.h"

// ---- initializePlugin --------------------------------------
MStatus initializePlugin(MObject obj)
{
    MFnPlugin plugin(obj, "ButterFlight", "0.1", "Any");

    MStatus status = plugin.registerCommand(
        BFSimulateCmd::kCommandName,
        BFSimulateCmd::creator,
        BFSimulateCmd::newSyntax);

    if (status != MS::kSuccess) {
        MGlobal::displayError("ButterFlight: Failed to register bfSimulate command.");
        return status;
    }

    // Register test commands
    status = plugin.registerCommand(
        BFTestAeroCmd::kCommandName,
        BFTestAeroCmd::creator);
    if (status != MS::kSuccess) {
        MGlobal::displayWarning("ButterFlight: Failed to register bfTestAero command.");
    }

    status = plugin.registerCommand(
        BFTestIntegrationCmd::kCommandName,
        BFTestIntegrationCmd::creator);
    if (status != MS::kSuccess) {
        MGlobal::displayWarning("ButterFlight: Failed to register bfTestIntegration command.");
    }

    MGlobal::displayInfo("ButterFlight v0.1 loaded.");
    return MS::kSuccess;
}

// ---- uninitializePlugin ------------------------------------
MStatus uninitializePlugin(MObject obj)
{
    MFnPlugin plugin(obj);

    plugin.deregisterCommand(BFTestIntegrationCmd::kCommandName);
    plugin.deregisterCommand(BFTestAeroCmd::kCommandName);

    MStatus status = plugin.deregisterCommand(BFSimulateCmd::kCommandName);
    if (status != MS::kSuccess) {
        MGlobal::displayError("ButterFlight: Failed to deregister bfSimulate command.");
        return status;
    }

    MGlobal::displayInfo("ButterFlight unloaded.");
    return MS::kSuccess;
}
