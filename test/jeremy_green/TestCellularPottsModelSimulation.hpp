/*

Copyright (c) 2005-2016, University of Oxford.
All rights reserved.

University of Oxford means the Chancellor, Masters and Scholars of the
University of Oxford, having an administrative office at Wellington
Square, Oxford OX1 2JD, UK.

This file is part of Chaste.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of the University of Oxford nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef TESTCELLULARPOTTSMODELSIMULATION_HPP_
#define TESTCELLULARPOTTSMODELSIMULATION_HPP_

#include <cxxtest/TestSuite.h>
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "FakePetscSetup.hpp"
#include "SmartPointers.hpp"
#include "CellsGenerator.hpp"
#include "GammaDistributedCellCycleModel.hpp"
#include "TransitCellProliferativeType.hpp"
#include "PottsMeshGenerator.hpp"
#include "OnLatticeSimulation.hpp"
#include "PottsBasedCellPopulation.hpp"
#include "VolumeConstraintPottsUpdateRule.hpp"
#include "SurfaceAreaConstraintPottsUpdateRule.hpp"
#include "AdhesionPottsUpdateRule.hpp"
#include "CellLabelWriter.hpp"

class TestPottsSimulation : public AbstractCellBasedWithTimingsTestSuite
{
public:

    void TestPottsModelLabelling() throw(Exception)
    {
        EXIT_IF_PARALLEL;

        // Create a square lattice with 50x50 sites, populated initially by 2x2 square elements, each comprising 4x4 lattice sites
        PottsMeshGenerator<2> generator(100, 2, 4, 100, 2, 4);
        PottsMesh<2>* p_mesh = generator.GetMesh();

        // Create a vector of proliferative cells and associate these with elements
        std::vector<CellPtr> cells;
        MAKE_PTR(TransitCellProliferativeType, p_type);
        CellsGenerator<GammaDistributedCellCycleModel, 2> cells_generator;
        cells_generator.GenerateBasicRandom(cells, p_mesh->GetNumElements(), p_type);
        for (unsigned i=0; i<cells.size(); i++)
        {
            // Set shape k and scale theta (mean = k*theta, variance = k*theta*theta)
            static_cast<GammaDistributedCellCycleModel*>(cells[i]->GetCellCycleModel())->SetShape(12.0);
            static_cast<GammaDistributedCellCycleModel*>(cells[i]->GetCellCycleModel())->SetScale(0.5);
        }

//        MAKE_PTR(CellLabel, p_label);
//        for (unsigned i = 0; i<cells.size(); i++)
//        {
//            if (RandomNumberGenerator::Instance()->ranf() < 0.5)
//            {
//                cells[i]->AddCellProperty(p_label);
//            }
//        }

        PottsBasedCellPopulation<2> cell_population(*p_mesh, cells);

        // Set "temperature" (defaults to 0.1)
        cell_population.SetTemperature(0.1);

        // Set the number of sweeps per time step (defaults to 1)
        cell_population.SetNumSweepsPerTimestep(1);

        // Specify cell labels to be output for visualization
        cell_population.AddCellWriter<CellLabelWriter>();

        // Configure simulation output
        OnLatticeSimulation<2> simulator(cell_population);
        simulator.SetOutputDirectory("PottsModelLabelling");
        simulator.SetEndTime(100.0);
        simulator.SetDt(0.1);
        simulator.SetSamplingTimestepMultiple(10);

        // Specify cell volume constraint 'energy'
        MAKE_PTR(VolumeConstraintPottsUpdateRule<2>, p_volume_constraint_update_rule);
        simulator.AddPottsUpdateRule(p_volume_constraint_update_rule);

        // Specify cell surface area constraint 'energy'
        MAKE_PTR(SurfaceAreaConstraintPottsUpdateRule<2>, p_surface_area_update_rule);
        simulator.AddPottsUpdateRule(p_surface_area_update_rule);

        // Specify cell-cell adhesion 'energy'
        MAKE_PTR(AdhesionPottsUpdateRule<2>, p_adhesion_update_rule);
        simulator.AddPottsUpdateRule(p_adhesion_update_rule);

        // Run simulation
        simulator.Solve();
    }

    /*
     * To visualize the results in Paraview, load the .pvd file and click apply.
     * Add box "Glyphs" to represent lattice sites. You will need to adjust the size so they don't overlap.
     * Select the "Display" tab and select "color by" cell index to see individual cells.
     * Add a "Threshold" filter, filter by cell type and make the lower threshold 0 or greater
     * (unoccupied lattice sites are labelled with -1). This will allow you to view only the cells.
     */
};

#endif /* TESTCELLULARPOTTSMODELSIMULATION_HPP_ */
