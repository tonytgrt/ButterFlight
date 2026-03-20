// ============================================================
// BFCurlNoise.cpp
// ButterFlight — Curl-noise vortex force computation
// Based on Chen et al. 2022, Section 5.2  (Bridson et al. 2007)
// Uses Improved Perlin Noise (Perlin 2002)
// ============================================================

#include "BFCurlNoise.h"
#include <cmath>

// ============================================================
// Perlin permutation table (standard reference table, doubled)
// ============================================================
const int BFCurlNoise::kPerm[512] = {
    151,160,137,91,90,15,131,13,201,95,96,53,194,233,7,225,
    140,36,103,30,69,142,8,99,37,240,21,10,23,190,6,148,
    247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,
    57,177,33,88,237,149,56,87,174,20,125,136,171,168,68,175,
    74,165,71,134,139,48,27,166,77,146,158,231,83,111,229,122,
    60,211,133,230,220,105,92,41,55,46,245,40,244,102,143,54,
    65,25,63,161,1,216,80,73,209,76,132,187,208,89,18,169,
    200,196,135,130,116,188,159,86,164,100,109,198,173,186,3,64,
    52,217,226,250,124,123,5,202,38,147,118,126,255,82,85,212,
    207,206,59,227,47,16,58,17,182,189,28,42,223,183,170,213,
    119,248,152,2,44,154,163,70,221,153,101,155,167,43,172,9,
    129,22,39,253,19,98,108,110,79,113,224,232,178,185,112,104,
    218,246,97,228,251,34,242,193,238,210,144,12,191,179,162,241,
    81,51,145,235,249,14,239,107,49,192,214,31,181,199,106,157,
    184,84,204,176,115,121,50,45,127,4,150,254,138,236,205,93,
    222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180,
    // Repeat for wrapping
    151,160,137,91,90,15,131,13,201,95,96,53,194,233,7,225,
    140,36,103,30,69,142,8,99,37,240,21,10,23,190,6,148,
    247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,
    57,177,33,88,237,149,56,87,174,20,125,136,171,168,68,175,
    74,165,71,134,139,48,27,166,77,146,158,231,83,111,229,122,
    60,211,133,230,220,105,92,41,55,46,245,40,244,102,143,54,
    65,25,63,161,1,216,80,73,209,76,132,187,208,89,18,169,
    200,196,135,130,116,188,159,86,164,100,109,198,173,186,3,64,
    52,217,226,250,124,123,5,202,38,147,118,126,255,82,85,212,
    207,206,59,227,47,16,58,17,182,189,28,42,223,183,170,213,
    119,248,152,2,44,154,163,70,221,153,101,155,167,43,172,9,
    129,22,39,253,19,98,108,110,79,113,224,232,178,185,112,104,
    218,246,97,228,251,34,242,193,238,210,144,12,191,179,162,241,
    81,51,145,235,249,14,239,107,49,192,214,31,181,199,106,157,
    184,84,204,176,115,121,50,45,127,4,150,254,138,236,205,93,
    222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180
};

// ============================================================
// fade  —  6t^5 - 15t^4 + 10t^3   (Perlin 2002 smoothstep)
// ============================================================
double BFCurlNoise::fade(double t)
{
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

// ============================================================
// lerp  —  linear interpolation
// ============================================================
double BFCurlNoise::lerp(double t, double a, double b)
{
    return a + t * (b - a);
}

// ============================================================
// grad  —  gradient function (Perlin 2002)
// Maps hash to one of 12 gradient directions.
// ============================================================
double BFCurlNoise::grad(int hash, double x, double y, double z)
{
    int h = hash & 15;
    double u = h < 8 ? x : y;
    double v = h < 4 ? y : (h == 12 || h == 14 ? x : z);
    return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
}

// ============================================================
// noise  —  Improved Perlin noise (Perlin 2002)
// ============================================================
double BFCurlNoise::noise(double x, double y, double z)
{
    // Find unit cube that contains the point
    int X = (int)std::floor(x) & 255;
    int Y = (int)std::floor(y) & 255;
    int Z = (int)std::floor(z) & 255;

    // Relative position within the cube
    x -= std::floor(x);
    y -= std::floor(y);
    z -= std::floor(z);

    // Fade curves
    double u = fade(x);
    double v = fade(y);
    double w = fade(z);

    // Hash coordinates of the 8 cube corners
    int A  = kPerm[X]     + Y;
    int AA = kPerm[A]     + Z;
    int AB = kPerm[A + 1] + Z;
    int B  = kPerm[X + 1] + Y;
    int BA = kPerm[B]     + Z;
    int BB = kPerm[B + 1] + Z;

    // Blended results from 8 corners of the cube
    return lerp(w,
        lerp(v,
            lerp(u, grad(kPerm[AA],     x,     y,     z),
                     grad(kPerm[BA],     x-1.0, y,     z)),
            lerp(u, grad(kPerm[AB],     x,     y-1.0, z),
                     grad(kPerm[BB],     x-1.0, y-1.0, z))),
        lerp(v,
            lerp(u, grad(kPerm[AA + 1], x,     y,     z-1.0),
                     grad(kPerm[BA + 1], x-1.0, y,     z-1.0)),
            lerp(u, grad(kPerm[AB + 1], x,     y-1.0, z-1.0),
                     grad(kPerm[BB + 1], x-1.0, y-1.0, z-1.0))));
}

// ============================================================
// computeForce  —  Curl-noise vortex force (Eq. 7)
//
//   F_vor = curl( s1(p/gx), s2(p/gy), s3(p/gz) ) * eta
//
// where s1, s2, s3 are Perlin noise fields with different seeds,
// and curl is computed via central finite differences.
// ============================================================
MVector BFCurlNoise::computeForce(const MPoint& position,
                                   double gainX,
                                   double gainY,
                                   double gainZ,
                                   double eta)
{
    // Scale position by gain parameters
    double px = position.x / gainX;
    double py = position.y / gainY;
    double pz = position.z / gainZ;

    // Helper: evaluate the three scalar noise fields at a point.
    // Each field uses a different seed offset to decorrelate them.
    // s1, s2, s3 form the potential vector field Ψ = (s1, s2, s3).
    auto s1 = [](double x, double y, double z) {
        return noise(x + kSeed1, y + kSeed1, z + kSeed1);
    };
    auto s2 = [](double x, double y, double z) {
        return noise(x + kSeed2, y + kSeed2, z + kSeed2);
    };
    auto s3 = [](double x, double y, double z) {
        return noise(x + kSeed3, y + kSeed3, z + kSeed3);
    };

    // Compute curl via central finite differences:
    //   curl(Ψ) = ( ∂s3/∂y - ∂s2/∂z,
    //               ∂s1/∂z - ∂s3/∂x,
    //               ∂s2/∂x - ∂s1/∂y )
    double eps = kEps;
    double inv2eps = 1.0 / (2.0 * eps);

    // ∂s3/∂y
    double ds3_dy = (s3(px, py + eps, pz) - s3(px, py - eps, pz)) * inv2eps;
    // ∂s2/∂z
    double ds2_dz = (s2(px, py, pz + eps) - s2(px, py, pz - eps)) * inv2eps;
    // ∂s1/∂z
    double ds1_dz = (s1(px, py, pz + eps) - s1(px, py, pz - eps)) * inv2eps;
    // ∂s3/∂x
    double ds3_dx = (s3(px + eps, py, pz) - s3(px - eps, py, pz)) * inv2eps;
    // ∂s2/∂x
    double ds2_dx = (s2(px + eps, py, pz) - s2(px - eps, py, pz)) * inv2eps;
    // ∂s1/∂y
    double ds1_dy = (s1(px, py + eps, pz) - s1(px, py - eps, pz)) * inv2eps;

    MVector curlF(ds3_dy - ds2_dz,
                  ds1_dz - ds3_dx,
                  ds2_dx - ds1_dy);

    return curlF * eta;
}
