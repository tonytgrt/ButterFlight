#pragma once
// ============================================================
// BFFlocking.h
// ButterFlight — Boids flocking forces for group simulation
// Based on Reynolds 1987: separation, alignment, cohesion
// ============================================================

#include "BFState.h"

#include <maya/MVector.h>
#include <vector>

class BFFlocking
{
public:
    // ---- Neighborhood radii (cm, Maya default unit) -------------
    double separationRadius = 50.0;  // cm — agents steer away within this range
    double neighborRadius   = 200.0; // cm — agents visible for alignment/cohesion

    // ---- Per-rule weights (acceleration scale) -----------------
    double separationWeight = 8.0;   // strong repulsion to prevent overlap
    double alignmentWeight  = 2.0;   // moderate heading matching
    double cohesionWeight   = 1.5;   // gentle pull toward group centroid

    // ---- Main interface ----------------------------------------

    /// Compute flocking accelerations for all agents.
    /// @param states  Array of per-butterfly simulation states.
    /// @return        One acceleration vector per agent (same indexing).
    std::vector<MVector> compute(const std::vector<BFState>& states) const;

    /// Compute flocking acceleration for a single agent given
    /// the full group state.  Useful for per-agent stepping.
    /// @param agentIdx  Index of the agent to evaluate.
    /// @param states    Array of all agent states.
    /// @return          Flocking acceleration for agent agentIdx.
    MVector computeForAgent(int agentIdx,
                            const std::vector<BFState>& states) const;

private:
    // ---- Per-rule force helpers ---------------------------------

    /// Separation: steer away from agents within separationRadius.
    /// Returns a repulsion acceleration that grows as 1/d for close
    /// neighbors, preventing collisions.
    MVector separation(int agentIdx,
                       const std::vector<BFState>& states) const;

    /// Alignment: steer toward the average heading of neighbors
    /// within neighborRadius.  Returns an acceleration that nudges
    /// this agent's velocity toward the local mean velocity.
    MVector alignment(int agentIdx,
                      const std::vector<BFState>& states) const;

    /// Cohesion: steer toward the centroid of neighbors within
    /// neighborRadius.  Returns an acceleration pointing from this
    /// agent toward the local center of mass.
    MVector cohesion(int agentIdx,
                     const std::vector<BFState>& states) const;
};
