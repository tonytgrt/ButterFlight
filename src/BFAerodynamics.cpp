// ============================================================
// BFAerodynamics.cpp
// ButterFlight — Quasi-steady aerodynamic force computation
// ============================================================

#include "BFAerodynamics.h"

#include <maya/MFnTransform.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MMatrix.h>
#include <maya/MGlobal.h>
#include <cmath>

// ============================================================
// liftCoeff  —  C_l(alpha)   (Chen et al. 2022, Eq. 5)
// ============================================================
double BFAerodynamics::liftCoeff(double alpha)
{
    // C_l(α) = −0.0095953 α² + 0.090635 α − 0.34182
    return -0.0095953 * alpha * alpha
           + 0.090635 * alpha
           - 0.34182;
}

// ============================================================
// dragCoeff  —  C_d(alpha)   (Chen et al. 2022, Eq. 6)
// ============================================================
double BFAerodynamics::dragCoeff(double alpha)
{
    // C_d(α) = −0.0000079518 α³ + 0.0011527 α² + 0.0063148 α + 0.51127
    double a2 = alpha * alpha;
    return -0.0000079518 * a2 * alpha
           + 0.0011527 * a2
           + 0.0063148 * alpha
           + 0.51127;
}

// ============================================================
// panelForce  —  combined lift + drag on one flat panel
// ============================================================
MVector BFAerodynamics::panelForce(const MVector& normal,
                                   double         area,
                                   const MVector& airVelocity)
{
    double speedSq = airVelocity * airVelocity;  // |V|^2
    if (speedSq < 1.0e-12) return MVector::zero;

    double speed = std::sqrt(speedSq);
    MVector Vhat = airVelocity / speed;           // unit air-velocity

    // Decompose air velocity into normal and tangential components
    double Vn_scalar = airVelocity * normal;      // signed projection onto normal
    MVector Vn = Vn_scalar * normal;              // normal component vector
    MVector Vt = airVelocity - Vn;                // tangential component vector

    double Vn_mag = std::fabs(Vn_scalar);
    double Vt_mag = Vt.length();

    // Angle of attack (radians)
    double alpha = std::atan2(Vn_mag, Vt_mag);

    // Dynamic pressure  q = 0.5 * rho * |V|^2
    double q = 0.5 * kRho * speedSq;

    // Coefficients
    double Cl = liftCoeff(alpha);
    double Cd = dragCoeff(alpha);

    // Drag direction: opposite to air velocity
    MVector dragDir = -Vhat;

    // Lift direction: component of panel normal perpendicular to V.
    // liftDir = normalize( n - (n · Vhat) * Vhat )
    MVector liftDir = normal - (normal * Vhat) * Vhat;
    double liftDirLen = liftDir.length();
    if (liftDirLen < 1.0e-12) {
        // Air flows exactly along the normal — no lift, only drag
        return q * area * Cd * dragDir;
    }
    liftDir /= liftDirLen;

    // Combined force
    return q * area * (Cl * liftDir + Cd * dragDir);
}

// ============================================================
// getWingNormal  —  joint's world-space Y axis
// ============================================================
MVector BFAerodynamics::getWingNormal(const MDagPath& wingJoint)
{
    MStatus status;
    MFnTransform xform(wingJoint, &status);
    if (status != MS::kSuccess) return MVector(0.0, 1.0, 0.0);

    MMatrix worldMat = wingJoint.inclusiveMatrix();
    // Y axis is the second column of the 4×4 matrix (rows 0-2)
    MVector yAxis(worldMat(1, 0), worldMat(1, 1), worldMat(1, 2));
    yAxis.normalize();
    return yAxis;
}

// ============================================================
// getWingSpanDir  —  joint's world-space X axis (base→tip)
// ============================================================
MVector BFAerodynamics::getWingSpanDir(const MDagPath& wingJoint)
{
    MStatus status;
    MFnTransform xform(wingJoint, &status);
    if (status != MS::kSuccess) return MVector(1.0, 0.0, 0.0);

    MMatrix worldMat = wingJoint.inclusiveMatrix();
    // X axis is the first column
    MVector xAxis(worldMat(0, 0), worldMat(0, 1), worldMat(0, 2));
    xAxis.normalize();
    return xAxis;
}

// ============================================================
// computeWingForce  —  flat-plate force for one wing
// ============================================================
MVector BFAerodynamics::computeWingForce(const MDagPath& wingJoint,
                                         const MVector&  bodyVelocity,
                                         const MVector&  wind,
                                         double          flapOmega,
                                         double          wingArea,
                                         double          meanRadius)
{
    MVector wingNormal  = getWingNormal(wingJoint);
    MVector wingSpanDir = getWingSpanDir(wingJoint);

    // The flap rotation axis is perpendicular to both the span and normal
    // (i.e. the joint's local Z axis), but for a simplified model the flap
    // angular velocity produces a linear velocity at the center of pressure
    // in the direction of the wing normal:
    //   V_flap = omega × r  ≈  flapOmega * meanRadius * wingNormal
    // (The cross product of the flap axis with the span direction gives
    //  approximately the normal direction for small feather/sweep angles.)
    MVector flapVelocity = flapOmega * meanRadius * wingNormal;

    // Air velocity relative to the wing panel center:
    //   V_air = wind − (bodyVelocity + flapVelocity)
    MVector airVelocity = wind - bodyVelocity - flapVelocity;

    return panelForce(wingNormal, wingArea, airVelocity);
}

// ============================================================
// computeTotalForce  —  sum of all four wings
// ============================================================
MVector BFAerodynamics::computeTotalForce(const BFSkeleton&   skeleton,
                                          const MVector&      bodyVelocity,
                                          const MVector&      wind,
                                          double              flapOmega,
                                          const BFWingParams& wp)
{
    if (!skeleton.valid) return MVector::zero;

    MVector total = MVector::zero;

    // Forewing L
    total += computeWingForce(skeleton.joints[kForewingL],
                              bodyVelocity, wind, flapOmega,
                              wp.forewingArea, wp.meanChordRadius);
    // Forewing R
    total += computeWingForce(skeleton.joints[kForewingR],
                              bodyVelocity, wind, flapOmega,
                              wp.forewingArea, wp.meanChordRadius);
    // Hindwing L
    total += computeWingForce(skeleton.joints[kHindwingL],
                              bodyVelocity, wind, flapOmega,
                              wp.hindwingArea, wp.meanChordRadius);
    // Hindwing R
    total += computeWingForce(skeleton.joints[kHindwingR],
                              bodyVelocity, wind, flapOmega,
                              wp.hindwingArea, wp.meanChordRadius);

    return total;
}
