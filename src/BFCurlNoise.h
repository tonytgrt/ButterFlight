#pragma once
// ============================================================
// BFCurlNoise.h
// ButterFlight — Curl-noise vortex force computation
// Based on Chen et al. 2022, Section 5.2  (Bridson et al. 2007)
// ============================================================

#include <maya/MVector.h>
#include <maya/MPoint.h>

class BFCurlNoise
{
public:
    // ---- Empirical parameters (Chen et al. 2022, Section 5.2) --
    static constexpr double kGainX = 22.0;
    static constexpr double kGainY =  5.5;
    static constexpr double kGainZ = 22.0;
    static constexpr double kEta   =  3.66;  // magnitude scaling

    // ---- Core Perlin noise (Perlin 2002, "Improving Noise") ----

    /// Evaluate improved Perlin noise at point (x,y,z).
    /// Returns value in approximately [-1, 1].
    static double noise(double x, double y, double z);

    // ---- Curl-noise vortex force (Eq. 7) -----------------------

    /// Compute the vortex force at a world-space position.
    /// @param position  Gravity center of the body (thorax position).
    /// @param gainX     Noise spatial scale X (default kGainX).
    /// @param gainY     Noise spatial scale Y (default kGainY).
    /// @param gainZ     Noise spatial scale Z (default kGainZ).
    /// @param eta       Force magnitude scaling (default kEta).
    /// @return Vortex force vector (N) in world space.
    static MVector computeForce(const MPoint& position,
                                double gainX = kGainX,
                                double gainY = kGainY,
                                double gainZ = kGainZ,
                                double eta   = kEta);

private:
    // Perlin noise helpers
    static double fade(double t);
    static double lerp(double t, double a, double b);
    static double grad(int hash, double x, double y, double z);

    // Permutation table (256 entries, doubled for wrapping)
    static const int kPerm[512];

    // Noise seeds (offsets) for the three scalar fields s1, s2, s3
    static constexpr double kSeed1 = 0.0;
    static constexpr double kSeed2 = 31.416;
    static constexpr double kSeed3 = 67.123;

    // Step size for finite-difference curl computation
    static constexpr double kEps = 1.0e-4;
};
