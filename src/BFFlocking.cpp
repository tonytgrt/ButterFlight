// ============================================================
// BFFlocking.cpp
// ButterFlight — Boids flocking forces for group simulation
// Based on Reynolds 1987: separation, alignment, cohesion
// ============================================================

#include "BFFlocking.h"

#include <cmath>

// ============================================================
// separation — repulsion from close neighbors
//
// For each neighbor j within separationRadius:
//   steer += (pos_i - pos_j) / |pos_i - pos_j|^2
//
// The 1/d^2 weighting pushes harder when agents are very close,
// providing a soft collision boundary.  The result is normalized
// by neighbor count to keep the magnitude stable regardless of
// local density.
// ============================================================
MVector BFFlocking::separation(int agentIdx,
                               const std::vector<BFState>& states) const
{
    MVector steer = MVector::zero;
    int count = 0;

    const MPoint& pos = states[agentIdx].position;

    for (int j = 0; j < (int)states.size(); ++j) {
        if (j == agentIdx) continue;

        MVector diff(pos.x - states[j].position.x,
                     pos.y - states[j].position.y,
                     pos.z - states[j].position.z);
        double dist = diff.length();

        if (dist < 1.0e-10 || dist > separationRadius) continue;

        // Weight by 1/d — closer agents produce stronger repulsion
        steer += diff * (1.0 / (dist * dist));
        ++count;
    }

    if (count > 0) {
        steer /= (double)count;
    }
    return steer;
}

// ============================================================
// alignment — velocity matching with neighbors
//
// Computes the average velocity of all neighbors within
// neighborRadius and returns a steering vector toward that
// average.  This causes the group to fly in roughly the same
// direction.
// ============================================================
MVector BFFlocking::alignment(int agentIdx,
                              const std::vector<BFState>& states) const
{
    MVector avgVelocity = MVector::zero;
    int count = 0;

    const MPoint& pos = states[agentIdx].position;

    for (int j = 0; j < (int)states.size(); ++j) {
        if (j == agentIdx) continue;

        MVector diff(pos.x - states[j].position.x,
                     pos.y - states[j].position.y,
                     pos.z - states[j].position.z);
        double dist = diff.length();

        if (dist > neighborRadius) continue;

        avgVelocity += states[j].velocity;
        ++count;
    }

    if (count == 0) return MVector::zero;

    avgVelocity /= (double)count;

    // Steer toward the average velocity (difference from current)
    return avgVelocity - states[agentIdx].velocity;
}

// ============================================================
// cohesion — attraction toward local center of mass
//
// Computes the centroid of all neighbors within neighborRadius
// and returns a steering vector from this agent toward that
// centroid.  This pulls stragglers back toward the group.
// ============================================================
MVector BFFlocking::cohesion(int agentIdx,
                             const std::vector<BFState>& states) const
{
    MVector centroid = MVector::zero;
    int count = 0;

    const MPoint& pos = states[agentIdx].position;

    for (int j = 0; j < (int)states.size(); ++j) {
        if (j == agentIdx) continue;

        MVector diff(pos.x - states[j].position.x,
                     pos.y - states[j].position.y,
                     pos.z - states[j].position.z);
        double dist = diff.length();

        if (dist > neighborRadius) continue;

        centroid += MVector(states[j].position.x,
                            states[j].position.y,
                            states[j].position.z);
        ++count;
    }

    if (count == 0) return MVector::zero;

    centroid /= (double)count;

    // Steer toward the centroid
    return MVector(centroid.x - pos.x,
                   centroid.y - pos.y,
                   centroid.z - pos.z);
}

// ============================================================
// computeForAgent — combined flocking acceleration for one agent
//
//   a_flock = w_sep * separation + w_ali * alignment + w_coh * cohesion
// ============================================================
MVector BFFlocking::computeForAgent(int agentIdx,
                                    const std::vector<BFState>& states) const
{
    if (states.size() < 2) return MVector::zero;

    MVector sep = separation(agentIdx, states);
    MVector ali = alignment(agentIdx, states);
    MVector coh = cohesion(agentIdx, states);

    return sep * separationWeight
         + ali * alignmentWeight
         + coh * cohesionWeight;
}

// ============================================================
// compute — flocking accelerations for all agents
// ============================================================
std::vector<MVector> BFFlocking::compute(
    const std::vector<BFState>& states) const
{
    const int n = (int)states.size();
    std::vector<MVector> accels(n, MVector::zero);

    for (int i = 0; i < n; ++i) {
        accels[i] = computeForAgent(i, states);
    }

    return accels;
}
