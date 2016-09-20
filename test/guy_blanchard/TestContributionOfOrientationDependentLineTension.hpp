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

#ifndef TESTCONTRIBUTIONOFORIENTATIONDEPENDENTLINETENSION_HPP_
#define TESTCONTRIBUTIONOFORIENTATIONDEPENDENTLINETENSION_HPP_

#include <cxxtest/TestSuite.h>
#include "CheckpointArchiveTypes.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "HoneycombVertexMeshGenerator.hpp"
#include "CellsGenerator.hpp"
#include "NoCellCycleModel.hpp"
#include "VertexBasedCellPopulation.hpp"
#include "StripeStatisticsWriter.hpp"
#include "BlanchardForce.hpp"
#include "ConstantTargetAreaModifier.hpp"
#include "OffLatticeSimulation.hpp"
#include "SmartPointers.hpp"
#include "FakePetscSetup.hpp"
#include "RandomForce.hpp"

class TestContributionOfOrientationDependentLineTension : public AbstractCellBasedWithTimingsTestSuite
{
public:

    void TestSimulation() throw (Exception)
    {
        bool include_orientation_dependence = false;
        bool use_combined_interface_line_tension = true;
        bool include_random_jiggling = false;
        bool check_internal_intersections = false;
        double time_step = 0.01; // 0.001
        double output_time_step = 1.0; // 0.1

        double end_time = 200.0;
        unsigned num_cells_wide = 14; // 28
        unsigned num_cells_high = 10; // 30

        // Initialise various singletons
        SimulationTime::Destroy();
        SimulationTime::Instance()->SetStartTime(0.0);
        CellPropertyRegistry::Instance()->Clear();
        CellId::ResetMaxCellId();

        // Generate a vertex mesh
        HoneycombVertexMeshGenerator honeycomb_generator(num_cells_wide, num_cells_high);
        MutableVertexMesh<2,2>* p_mesh = honeycomb_generator.GetMesh();
        p_mesh->SetCheckForInternalIntersections(check_internal_intersections);

        // Create some non-proliferating cells
        std::vector<CellPtr> cells;
        CellsGenerator<NoCellCycleModel, 2> cells_generator;
        cells_generator.GenerateBasic(cells, p_mesh->GetNumElements());

        // Bestow cell stripe identities
        for (unsigned i=0; i<cells.size(); i++)
        {
            unsigned row = i/num_cells_wide;
            unsigned col = i%num_cells_wide;

            if (row%4 == 0)
            {
                if ((col%7 == 0) || (col%7 == 4))      { cells[i]->GetCellData()->SetItem("stripe", 1); }
                else if ((col%7 == 1) || (col%7 == 5)) { cells[i]->GetCellData()->SetItem("stripe", 2); }
                else if ((col%7 == 2) || (col%7 == 6)) { cells[i]->GetCellData()->SetItem("stripe", 3); }
                else                                   { cells[i]->GetCellData()->SetItem("stripe", 4); }
            }
            else if (row%4 == 1)
            {
                if ((col%7 == 0) || (col%7 == 3))      { cells[i]->GetCellData()->SetItem("stripe", 1); }
                else if ((col%7 == 1) || (col%7 == 4)) { cells[i]->GetCellData()->SetItem("stripe", 2); }
                else if (col%7 == 5)                   { cells[i]->GetCellData()->SetItem("stripe", 3); }
                else                                   { cells[i]->GetCellData()->SetItem("stripe", 4); }
            }
            else if (row%4 == 2)
            {
                if ((col%7 == 0) || (col%7 == 4))      { cells[i]->GetCellData()->SetItem("stripe", 1); }
                else if ((col%7 == 1) || (col%7 == 5)) { cells[i]->GetCellData()->SetItem("stripe", 2); }
                else if (col%7 == 2)                   { cells[i]->GetCellData()->SetItem("stripe", 3); }
                else                                   { cells[i]->GetCellData()->SetItem("stripe", 4); }
            }
            else
            {
                if ((col%7 == 0) || (col%7 == 3))      { cells[i]->GetCellData()->SetItem("stripe", 1); }
                else if ((col%7 == 1) || (col%7 == 4)) { cells[i]->GetCellData()->SetItem("stripe", 2); }
                else if ((col%7 == 2) || (col%7 == 5)) { cells[i]->GetCellData()->SetItem("stripe", 3); }
                else                                   { cells[i]->GetCellData()->SetItem("stripe", 4); }
            }
        }

        // Create a cell population that associates the cells with the vertex mesh
        VertexBasedCellPopulation<2> cell_population(*p_mesh, cells);
        cell_population.SetOutputResultsForChasteVisualizer(false);
        cell_population.SetOutputCellRearrangementLocations(false);
        cell_population.AddPopulationWriter<StripeStatisticsWriter>();

        // Create a simulation using the cell population
        OffLatticeSimulation<2> simulation(cell_population);
        simulation.SetOutputDirectory("TestContributionOfOrientationDependentLineTension");
        simulation.SetEndTime(end_time);

        simulation.SetDt(time_step);
        unsigned output_time_step_multiple = (unsigned) (output_time_step/time_step);
        simulation.SetSamplingTimestepMultiple(output_time_step_multiple);

        // Create the appropriate force law(s) for the specified geometry
        MAKE_PTR(BlanchardForce<2>, p_force);
        p_force->SetNumStripes(4);
        p_force->SetAreaElasticityParameter(5.0); // 10.0     // 20.0
        p_force->SetPerimeterContractilityParameter(1.0);      // 2.0
        p_force->SetHomotypicLineTensionParameter(1.0);        // 1.0
        p_force->SetHeterotypicLineTensionParameter(2.0);      // 2.0
        p_force->SetSupercontractileLineTensionParameter(4.0); // 4.0
        p_force->SetBoundaryLineTensionParameter(1.0);         // 1.0

        p_force->SetIncludeOrientationDependence(include_orientation_dependence);
        if (include_orientation_dependence)
        {
            std::set<std::set<unsigned> > original_edges;
            for (unsigned elem_index=0; elem_index<p_mesh->GetNumElements(); elem_index++)
            {
                std::set<unsigned> neighbours = p_mesh->GetNeighbouringElementIndices(elem_index);

                for (std::set<unsigned>::iterator iter = neighbours.begin(); iter != neighbours.end(); ++iter)
                {
                    std::set<unsigned> this_edge;
                    this_edge.insert(elem_index);
                    this_edge.insert(*iter);

                    original_edges.insert(this_edge);
                }
            }
            p_force->SetOriginalEdges(original_edges);
        }

        p_force->SetUseCombinedInterfaceLineTension(use_combined_interface_line_tension);

        simulation.AddForce(p_force);

        if (include_random_jiggling)
        {
            MAKE_PTR(RandomForce<2>, p_random_force);
            p_random_force->SetDiffusionConstant(0.01);
            simulation.AddForce(p_random_force);
        }

        // Pass in a target area modifier (needed, but not used)
        MAKE_PTR(ConstantTargetAreaModifier<2>, p_growth_modifier);
        simulation.AddSimulationModifier(p_growth_modifier);

        // Run simulation
        simulation.Solve();
    }
};

#endif /*TESTCONTRIBUTIONOFORIENTATIONDEPENDENTLINETENSION_HPP_*/
