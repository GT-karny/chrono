// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Co-simulation FMU encapsulating a terrain system.
//
// This terrain FMU must be co-simulated with a vehicle system (or tire system)
// which provides the current query point (of type ChVector).
//
// This terrain FMU defines continuous output variables for:
//   - terrain height at query point (Real)
//   - terrain normal at query point (Vec3)
//   - terrain friction at query point (Real)
//
// =============================================================================

#pragma once

#include <string>
#include <vector>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_fmi/fmi2/ChFmuToolsExport.h"

class FmuComponent : public chrono::fmi2::FmuChronoComponentBase {
  public:
    FmuComponent(fmi2String instanceName,
                 fmi2Type fmuType,
                 fmi2String fmuGUID,
                 fmi2String fmuResourceLocation,
                 const fmi2CallbackFunctions* functions,
                 fmi2Boolean visible,
                 fmi2Boolean loggingOn);
    ~FmuComponent() {}

    /// Advance dynamics.
    virtual fmi2Status doStepIMPL(fmi2Real currentCommunicationPoint,
                                  fmi2Real communicationStepSize,
                                  fmi2Boolean noSetFMUStatePriorToCurrentPoint) override;

  private:
    virtual fmi2Status enterInitializationModeIMPL() override;
    virtual fmi2Status exitInitializationModeIMPL() override;

    virtual void preModelDescriptionExport() override;
    virtual void postModelDescriptionExport() override;

    virtual bool is_cosimulation_available() const override { return true; }
    virtual bool is_modelexchange_available() const override { return false; }

    void CreateTerrain();
    void CalculateTerrainOutputs();

    std::shared_ptr<chrono::vehicle::ChTerrain> terrain;  ///< underlying terrain object
    chrono::ChSystemSMC sys;                              ///< containing system

    // FMU I/O parameters
    std::string out_path;  ///< output directory

    // FMU parameters
    std::string terrain_type;  ///< Terrain type: "Flat" or "RigidMesh"
    std::string json_file;     ///< JSON specification file (for RigidMesh)
    std::string obj_file;      ///< OBJ file (for RigidMesh, explicit)
    double friction;           ///< Friction coefficient (for Flat or default)
    double step_size;          ///< integration step size

    // FMU inputs (from vehicle/tire)
    chrono::ChVector3d query_point;  ///< query point location

    // FMU outputs (to vehicle/tire)
    double h;                    ///< terrain height at query point
    chrono::ChVector3d n;        ///< terrain normal at query point
    double mu;                   ///< terrain friction at query point
};
