#pragma once
// ============================================================
// BFAerodynamics.h
// ButterFlight — Quasi-steady aerodynamic force computation
// Based on Chen et al. 2022, Section 5.1  (Ellington 1984a)
// ============================================================

#include <maya/MVector.h>
#include <maya/MDagPath.h>
#include "BFState.h"

class BFAerodynamics
{
public:
    // ---- Physical constants --------------------------------
    static constexpr double kRho = 1.225;  // air density (kg/m^3) at sea level

    // ---- Empirical coefficient polynomials (Eqs. 5–6) ------

    /// Lift coefficient C_l(alpha).  alpha in radians.
    static double liftCoeff(double alpha);

    /// Drag coefficient C_d(alpha).  alpha in radians.
    static double dragCoeff(double alpha);

    // ---- Core per-panel computation ------------------------

    /// Combined lift + drag force on a single flat panel.
    /// @param normal      Panel outward normal (unit vector).
    /// @param area        Panel area (m^2).
    /// @param airVelocity Velocity of air relative to the panel (m/s).
    /// @return Net aerodynamic force vector (N) in world space.
    static MVector panelForce(const MVector& normal,
                              double         area,
                              const MVector& airVelocity);

    // ---- Joint orientation helpers -------------------------

    /// Wing panel normal from the joint's world-space Y axis.
    static MVector getWingNormal(const MDagPath& wingJoint);

    /// Wing span (base-to-tip) direction from the joint's world-space X axis.
    static MVector getWingSpanDir(const MDagPath& wingJoint);

    // ---- High-level force aggregation ----------------------

    /// Compute aerodynamic force for one wing (flat-plate model).
    /// @param wingJoint      DAG path of the wing joint.
    /// @param bodyVelocity   World-space velocity of the thorax (m/s).
    /// @param wind           World-space wind velocity (m/s).
    /// @param flapOmega      Wing flapping angular velocity (rad/s, scalar about flap axis).
    /// @param wingArea       Wing panel area (m^2).
    /// @param meanRadius     Mean distance from joint to center of pressure (m).
    static MVector computeWingForce(const MDagPath& wingJoint,
                                    const MVector&  bodyVelocity,
                                    const MVector&  wind,
                                    double          flapOmega,
                                    double          wingArea,
                                    double          meanRadius);

    /// Sum aerodynamic forces from all four wings.
    /// @param skeleton       Resolved butterfly skeleton.
    /// @param bodyVelocity   World-space velocity of the thorax (m/s).
    /// @param wind           World-space wind velocity (m/s).
    /// @param flapOmega      Flapping angular velocity (rad/s) — same for all four wings
    ///                       (bilateral symmetry assumption from design doc).
    /// @param wingParams     Per-species wing area and geometry.
    static MVector computeTotalForce(const BFSkeleton&  skeleton,
                                     const MVector&     bodyVelocity,
                                     const MVector&     wind,
                                     double             flapOmega,
                                     const BFWingParams& wingParams);
};
