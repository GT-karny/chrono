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
// =============================================================================

#include <cassert>
#include <algorithm>

#include "FMU_Terrain.h"

#include "chrono/physics/ChContactMaterialSMC.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::fmi2;

// -----------------------------------------------------------------------------

// Create an instance of this FMU
fmu_forge::fmi2::FmuComponentBase* fmu_forge::fmi2::fmi2InstantiateIMPL(fmi2String instanceName,
                                                                        fmi2Type fmuType,
                                                                        fmi2String fmuGUID,
                                                                        fmi2String fmuResourceLocation,
                                                                        const fmi2CallbackFunctions* functions,
                                                                        fmi2Boolean visible,
                                                                        fmi2Boolean loggingOn) {
    return new FmuComponent(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn);
}

// -----------------------------------------------------------------------------

FmuComponent::FmuComponent(fmi2String instanceName,
                           fmi2Type fmuType,
                           fmi2String fmuGUID,
                           fmi2String fmuResourceLocation,
                           const fmi2CallbackFunctions* functions,
                           fmi2Boolean visible,
                           fmi2Boolean loggingOn)
    : FmuChronoComponentBase(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn) {
    // Initialize FMU type
    initializeType(fmuType);

    // Set initial/default values for FMU variables
    step_size = 1e-3;
    out_path = ".";
    
    terrain_type = "Flat";
    json_file = "";
    obj_file = "";
    friction = 0.8;

    query_point = ChVector3d(0.0);
    h = 0;
    n = ChVector3d(0, 0, 1);
    mu = 0.8;

    // Set FIXED PARAMETERS for this FMU
    AddFmuVariable(&terrain_type, "terrain_type", FmuVariable::Type::String, "1", "terrain type (Flat, RigidMesh)",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                      //
    AddFmuVariable(&json_file, "json_file", FmuVariable::Type::String, "1", "terrain JSON file",                     //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                      //
    AddFmuVariable(&obj_file, "obj_file", FmuVariable::Type::String, "1", "terrain OBJ file",                        //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                      //
    AddFmuVariable(&friction, "friction", FmuVariable::Type::Real, "1", "friction coefficient",                      //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                      //
    AddFmuVariable(&step_size, "step_size", FmuVariable::Type::Real, "s", "integration step size",                   //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                      //
    AddFmuVariable(&out_path, "out_path", FmuVariable::Type::String, "1", "output directory",                        //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                      //

    // Set CONTINOUS INPUTS for this FMU
    AddFmuVecVariable(query_point, "query_point", "m", "query point",                                   //
                      FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);     //

    // Set CONTINUOUS OUTPUTS for this FMU
    AddFmuVariable(&h, "height", FmuVariable::Type::Real, "m", "terrain height",                        //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,        //
                   FmuVariable::InitialType::exact);                                                    //
    AddFmuVecVariable(n, "normal", "1", "terrain normal",                                               //
                      FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,     //
                      FmuVariable::InitialType::exact);                                                 //
    AddFmuVariable(&mu, "mu", FmuVariable::Type::Real, "1", "terrain friction",                        //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,        //
                   FmuVariable::InitialType::exact);                                                    //

    // Specify functions to calculate FMU outputs (at end of step)
    AddPostStepFunction([this]() { this->CalculateTerrainOutputs(); });
}

void FmuComponent::CreateTerrain() {
    std::cout << "Create terrain FMU" << std::endl;
    std::cout << " Type: " << terrain_type << std::endl;

    if (terrain_type == "Flat") {
        auto flat_terrain = std::make_shared<FlatTerrain>(0.0, (float)friction);
        terrain = flat_terrain;
    } else if (terrain_type == "RigidMesh") {
        std::shared_ptr<RigidTerrain> rigid_terrain;

        if (!json_file.empty()) {
             std::cout << " Loading from JSON: " << json_file << std::endl;
             rigid_terrain = std::make_shared<RigidTerrain>(&sys, json_file);
             rigid_terrain->Initialize();
        } else if (!obj_file.empty()) {
            std::cout << " Loading from OBJ: " << obj_file << std::endl;
            rigid_terrain = std::make_shared<RigidTerrain>(&sys);
            
            auto patch_mat = chrono_types::make_shared<ChContactMaterialSMC>();
            patch_mat->SetFriction((float)friction);
            patch_mat->SetRestitution(0.01f);
            
            auto patch = rigid_terrain->AddPatch(patch_mat, ChCoordsys<>(), obj_file);
            rigid_terrain->Initialize();
        } else {
             std::cerr << "TerrainFMU Error: RigidMesh selected but no JSON or OBJ file provided." << std::endl;
             // Fallback to flat?
             auto flat_terrain = std::make_shared<FlatTerrain>(0.0, (float)friction);
             terrain = flat_terrain;
             return;
        }
        terrain = rigid_terrain;
    } else {
        std::cerr << "TerrainFMU Error: Unknown terrain type " << terrain_type << ". Defaulting to Flat." << std::endl;
        auto flat_terrain = std::make_shared<FlatTerrain>(0.0, (float)friction);
        terrain = flat_terrain;
    }
}

void FmuComponent::CalculateTerrainOutputs() {
    if (terrain) {
        h = terrain->GetHeight(query_point);
        n = terrain->GetNormal(query_point);
        mu = (double)terrain->GetCoefficientFriction(query_point);
    }
}

void FmuComponent::preModelDescriptionExport() {}

void FmuComponent::postModelDescriptionExport() {}

fmi2Status FmuComponent::enterInitializationModeIMPL() {
    return fmi2Status::fmi2OK;
}

fmi2Status FmuComponent::exitInitializationModeIMPL() {
    CreateTerrain();
    // Initial calculation
    CalculateTerrainOutputs();
    return fmi2Status::fmi2OK;
}

fmi2Status FmuComponent::doStepIMPL(fmi2Real currentCommunicationPoint,
                                    fmi2Real communicationStepSize,
                                    fmi2Boolean noSetFMUStatePriorToCurrentPoint) {
    while (m_time < currentCommunicationPoint + communicationStepSize) {
        fmi2Real dt = std::min((currentCommunicationPoint + communicationStepSize - m_time),
                              std::min(communicationStepSize, step_size));

        // Nothing to advance for static terrain, just update time
        // If we had moving terrain or deformable terrain, we would call terrain->Advance(h) etc.
        m_time += dt;
    }

    return fmi2Status::fmi2OK;
}
