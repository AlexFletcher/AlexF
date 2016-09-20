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

#ifndef TESTCONTRACTILITYVERSUSSUPERCONTRACTILITY_HPP_
#define TESTCONTRACTILITYVERSUSSUPERCONTRACTILITY_HPP_

#include <cxxtest/TestSuite.h>
#include "CheckpointArchiveTypes.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "HoneycombVertexMeshGenerator.hpp"
#include "CellsGenerator.hpp"
#include "NoCellCycleModel.hpp"
#include "CellLabel.hpp"
#include "RandomForce.hpp" ///
#include "CellLabelWriter.hpp"
#include "VertexBasedCellPopulation.hpp"
#include "NagaiHondaMultipleLabelsForce.hpp"
#include "SimpleTargetAreaModifier.hpp"
#include "OffLatticeSimulation.hpp"
#include "SmartPointers.hpp"
#include "FakePetscSetup.hpp"

class TestContractilityVersusSupercontractility : public AbstractCellBasedWithTimingsTestSuite
{
public:

    void TestSupercontractilityWithRegularInitialGeometry() throw (Exception)
    {
        bool use_exponential_line_tension = false;
        bool set_for_internal_intersections = false; // default value; if true, the sim runs more slowly
        bool jiggle_vertices = false;
        double end_time = 30.0;

        // Generate a vertex mesh
        unsigned num_cells_wide = 14;
        unsigned num_cells_high = 10;
        HoneycombVertexMeshGenerator honeycomb_generator(num_cells_wide, num_cells_high);
        MutableVertexMesh<2,2>* p_mesh = honeycomb_generator.GetMesh();
        p_mesh->SetCheckForInternalIntersections(set_for_internal_intersections);

        // Create some non-proliferating cells
        std::vector<CellPtr> cells;
        CellsGenerator<NoCellCycleModel, 2> cells_generator;
        cells_generator.GenerateBasic(cells, p_mesh->GetNumElements());

        // Label each cell to its identity within each PS with 4 stripes
        MAKE_PTR_ARGS(CellLabel, p_label_1, (10));
        MAKE_PTR_ARGS(CellLabel, p_label_2, (11));
        MAKE_PTR_ARGS(CellLabel, p_label_3, (12));
        MAKE_PTR_ARGS(CellLabel, p_label_4, (13));
        for (unsigned i=0; i<cells.size(); i++)
        {
            unsigned row = i/num_cells_wide;
            unsigned col = i%num_cells_wide;

            if (row%4 == 0)
            {
                if ((col%7 == 0) || (col%7 == 4))      { cells[i]->AddCellProperty(p_label_1); }
                else if ((col%7 == 1) || (col%7 == 5)) { cells[i]->AddCellProperty(p_label_2); }
                else if ((col%7 == 2) || (col%7 == 6)) { cells[i]->AddCellProperty(p_label_3); }
                else                                   { cells[i]->AddCellProperty(p_label_4); }
            }
            else if (row%4 == 1)
            {
                if ((col%7 == 0) || (col%7 == 3))      { cells[i]->AddCellProperty(p_label_1); }
                else if ((col%7 == 1) || (col%7 == 4)) { cells[i]->AddCellProperty(p_label_2); }
                else if (col%7 == 5)                   { cells[i]->AddCellProperty(p_label_3); }
                else                                   { cells[i]->AddCellProperty(p_label_4); }
            }
            else if (row%4 == 2)
            {
                if ((col%7 == 0) || (col%7 == 4))      { cells[i]->AddCellProperty(p_label_1); }
                else if ((col%7 == 1) || (col%7 == 5)) { cells[i]->AddCellProperty(p_label_2); }
                else if (col%7 == 2)                   { cells[i]->AddCellProperty(p_label_3); }
                else                                   { cells[i]->AddCellProperty(p_label_4); }
            }
            else
            {
                if ((col%7 == 0) || (col%7 == 3))      { cells[i]->AddCellProperty(p_label_1); }
                else if ((col%7 == 1) || (col%7 == 4)) { cells[i]->AddCellProperty(p_label_2); }
                else if ((col%7 == 2) || (col%7 == 5)) { cells[i]->AddCellProperty(p_label_3); }
                else                                   { cells[i]->AddCellProperty(p_label_4); }
            }
        }

        // Create a cell population that associates the cells with the vertex mesh
        VertexBasedCellPopulation<2> cell_population(*p_mesh, cells);
        cell_population.SetOutputResultsForChasteVisualizer(false);
        cell_population.AddCellWriter<CellLabelWriter>();

        // Create a simulation using the cell population
        OffLatticeSimulation<2> simulator(cell_population);
        simulator.SetOutputDirectory("TestSupercontractilityWithRegularInitialGeometry");
        simulator.SetEndTime(end_time);

        // Specify a small time step, but only output results to VTK every hour
        double time_step = 0.001;
        unsigned output_time_step_multiple = (unsigned) (0.1/time_step);//(unsigned) (1.0/time_step); ///\todo
        simulator.SetDt(time_step);
        simulator.SetSamplingTimestepMultiple(output_time_step_multiple);

        // Create the appropriate force law(s) for the specified geometry
        MAKE_PTR(NagaiHondaMultipleLabelsForce<2>, p_label_force);
        p_label_force->SetUseExponentialLineTension(use_exponential_line_tension);
        p_label_force->SetNagaiHondaDeformationEnergyParameter(10.0);
        p_label_force->SetNagaiHondaMembraneSurfaceEnergyParameter(1.0); // Could use to keep cells more isotropic. Currently no obvious need.
        p_label_force->SetCellBoundaryAdhesionParameter(1.0);      //1.0 // Boundary behaves oddly if this is not set, but probably constrains the deformation of the tissue?
        p_label_force->SetHeterotypicCellAdhesionParameter(2.0);   //1.0 // Reduce heterotypic interfaces with this penalty. This uses a multiplier depending on neighbour label difference.
        p_label_force->SetHomotypicCellAdhesionParameter(1.0);     //0.0 // No penalty to within cell type interfaces
        p_label_force->SetLambdaParameter(2.0);                    //1.0 // Penalty for all heterotypic interfaces, particularly the shorter they are, to mimic concentration of myosin with reduction in length, driving to neighbour exchange (hopefully). This also needs multiplier as above.
        simulator.AddForce(p_label_force);

        // Pass in a target area modifier (needed, but not used)
        MAKE_PTR(SimpleTargetAreaModifier<2>, p_growth_modifier);
        simulator.AddSimulationModifier(p_growth_modifier);

        if (jiggle_vertices)
        {
            MAKE_PTR(RandomForce<2>, p_random_force);
            p_random_force->SetDiffusionConstant(0.01);
            simulator.AddForce(p_random_force);
        }

        // Run simulation
        simulator.Solve();
    }
};

#endif /*TESTCONTRACTILITYVERSUSSUPERCONTRACTILITY_HPP_*/
