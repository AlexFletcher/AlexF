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

#ifndef TESTSIMULATIONSFORGBEPAPER_HPP_
#define TESTSIMULATIONSFORGBEPAPER_HPP_

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

class TestSimulationsForGbePaper : public AbstractCellBasedWithTimingsTestSuite
{
public:

    void TestParameterSweeping() throw (Exception)
    {
        unsigned num_values = 4;

        OutputFileHandler results_handler("TestSimulationsForGbePaper", false);
        out_stream results_file = results_handler.OpenOutputFile("results.dat");

        bool set_for_internal_intersections = false;
        double end_time = 10.0;
        unsigned num_cells_wide = 14;
        unsigned num_cells_high = 10;

        double heterotypic_values[3] = {1.0, 1.5, 2.0};

        for (unsigned index1=0; index1<3; index1++)
        {
            double heterotypic_line_tension_parameter = heterotypic_values[index1];

            double supercontractile_values[7] = {1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
            for (unsigned index2=0; index2<7; index2++)
            {
                double supercontractile_line_tension_parameter = supercontractile_values[index2];

                // Initialise various singletons
                SimulationTime::Destroy();
                SimulationTime::Instance()->SetStartTime(0.0);
                CellPropertyRegistry::Instance()->Clear();
                CellId::ResetMaxCellId();

                // Generate a vertex mesh
                HoneycombVertexMeshGenerator honeycomb_generator(num_cells_wide, num_cells_high);
                MutableVertexMesh<2,2>* p_mesh = honeycomb_generator.GetMesh();
                p_mesh->SetCheckForInternalIntersections(set_for_internal_intersections);

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
                simulation.SetOutputDirectory("TestParameterSweeping");
                simulation.SetEndTime(end_time);

                // Specify a small time step, but only output results to VTK every hour
                double time_step = 0.001;
                unsigned output_time_step_multiple = (unsigned) (end_time/time_step); //(unsigned) (0.1/time_step);
                simulation.SetDt(time_step);
                simulation.SetSamplingTimestepMultiple(output_time_step_multiple);

                // Create the appropriate force law(s) for the specified geometry
                MAKE_PTR(BlanchardForce<2>, p_force);
                p_force->SetNumStripes(4);
                p_force->SetAreaElasticityParameter(20.0);
                p_force->SetPerimeterContractilityParameter(2.0);
                p_force->SetHomotypicLineTensionParameter(1.0);
                p_force->SetHeterotypicLineTensionParameter(heterotypic_line_tension_parameter);
                p_force->SetSupercontractileLineTensionParameter(supercontractile_line_tension_parameter); // Usually twice the value passed to SetHeterotypicLineTensionParamete()
                p_force->SetBoundaryLineTensionParameter(1.0);

                simulation.AddForce(p_force);

                // Pass in a target area modifier (needed, but not used)
                MAKE_PTR(ConstantTargetAreaModifier<2>, p_growth_modifier);
                simulation.AddSimulationModifier(p_growth_modifier);

                // Run simulation
                simulation.Solve();

                // Initialise helper variables
                double total_num_edges = 0.0;
                double total_edges_length = 0.0;
                double mismatch_one_num_edges = 0.0;
                double mismatch_one_boundary_length = 0.0;
                double mismatch_two_num_edges = 0.0;
                double mismatch_two_boundary_length = 0.0;

                // Iterate over cells and compute summary statistics
                VertexBasedCellPopulation<2>* p_cell_population = static_cast<VertexBasedCellPopulation<2>*>(&(simulation.rGetCellPopulation()));
                for (VertexBasedCellPopulation<2>::Iterator cell_iter = p_cell_population->Begin();
                     cell_iter != p_cell_population->End();
                     ++cell_iter)
                {
                    // Find this cell's stripe identity
                    unsigned cell_stripe_identity = cell_iter->GetCellData()->GetItem("stripe");

                    // Get the set of neighbouring element indices
                    unsigned elem_index = p_cell_population->GetLocationIndexUsingCell(*cell_iter);
                    std::set<unsigned> neighbour_elem_indices = p_cell_population->rGetMesh().GetNeighbouringElementIndices(elem_index);

                    // Iterate over these neighbours
                    for (std::set<unsigned>::iterator neighbour_iter = neighbour_elem_indices.begin();
                         neighbour_iter != neighbour_elem_indices.end();
                         ++neighbour_iter)
                    {
                        // Get the length of the edge shared with this neighbour
                        unsigned neighbour_index = *neighbour_iter;
                        double edge_length = p_cell_population->rGetMesh().GetEdgeLength(elem_index, neighbour_index);

                        total_edges_length += edge_length;
                        total_num_edges += 1.0;

                        // Find this neighbour's stripe identity
                        CellPtr p_neighbour = p_cell_population->GetCellUsingLocationIndex(*neighbour_iter);
                        unsigned neighbour_stripe_identity = p_neighbour->GetCellData()->GetItem("stripe");

                        unsigned num_stripes = 4; ///\todo remove hardcoding
                        unsigned mismatch = abs(cell_stripe_identity - neighbour_stripe_identity);
                        if (mismatch > num_stripes/2)
                        {
                            mismatch = num_stripes - mismatch;
                        }

                        if (mismatch == 1)
                        {
                            mismatch_one_num_edges += 1.0;
                            mismatch_one_boundary_length += edge_length;
                        }
                        else if (mismatch == 2)
                        {
                            mismatch_two_num_edges += 1.0;
                            mismatch_two_boundary_length += edge_length;
                        }
                        else
                        {
                            assert(mismatch == 0);
                        }
                    }
                }

                // We have counted each cell-cell edge twice
                total_num_edges *= 0.5;
                total_edges_length *= 0.5;
                mismatch_one_num_edges *= 0.5;
                mismatch_one_boundary_length *= 0.5;
                mismatch_two_num_edges *= 0.5;
                mismatch_two_boundary_length *= 0.5;

                // Output summary statistics to results file
                (*results_file) << heterotypic_line_tension_parameter << "\t"
                                << supercontractile_line_tension_parameter << "\t"
                                << total_num_edges << "\t"
                                << total_edges_length << "\t"
                                << mismatch_one_num_edges << "\t"
                                << mismatch_one_boundary_length
                                << "\t" << mismatch_two_num_edges
                                << "\t" << mismatch_two_boundary_length
                                << "\n";
            }
        }

        // Tidy up
        results_file->close();
    }

    void NOTestSingleSimulation() throw (Exception)
    {
        bool set_for_internal_intersections = false;
        double end_time = 30.0;
        unsigned num_cells_wide = 14;
        unsigned num_cells_high = 10;

        double heterotypic_line_tension_parameter = 8.0;
        double supercontractile_line_tension_parameter = 9.0;

        // Initialise various singletons
        SimulationTime::Destroy();
        SimulationTime::Instance()->SetStartTime(0.0);
        CellPropertyRegistry::Instance()->Clear();
        CellId::ResetMaxCellId();

        // Generate a vertex mesh
        HoneycombVertexMeshGenerator honeycomb_generator(num_cells_wide, num_cells_high);
        MutableVertexMesh<2,2>* p_mesh = honeycomb_generator.GetMesh();
        p_mesh->SetCheckForInternalIntersections(set_for_internal_intersections);

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
        simulation.SetOutputDirectory("TestSingleSimulation");
        simulation.SetEndTime(end_time);

        // Specify a small time step, but only output results to VTK every hour
        double time_step = 0.001;
        unsigned output_time_step_multiple = (unsigned) (0.1/time_step);
        simulation.SetDt(time_step);
        simulation.SetSamplingTimestepMultiple(output_time_step_multiple);

        // Create the appropriate force law(s) for the specified geometry
        MAKE_PTR(BlanchardForce<2>, p_force);
        p_force->SetNumStripes(4);
        p_force->SetAreaElasticityParameter(20.0);
        p_force->SetPerimeterContractilityParameter(2.0);
        p_force->SetHomotypicLineTensionParameter(1.0);
        p_force->SetHeterotypicLineTensionParameter(heterotypic_line_tension_parameter);
        p_force->SetSupercontractileLineTensionParameter(supercontractile_line_tension_parameter); // Usually twice the value passed to SetHeterotypicLineTensionParamete()
        p_force->SetBoundaryLineTensionParameter(1.0);

        simulation.AddForce(p_force);

        // Pass in a target area modifier (needed, but not used)
        MAKE_PTR(ConstantTargetAreaModifier<2>, p_growth_modifier);
        simulation.AddSimulationModifier(p_growth_modifier);

//        MAKE_PTR(RandomForce<2>, p_random_force);
//        p_random_force->SetDiffusionConstant(0.01);
//        simulation.AddForce(p_random_force);

        // Run simulation
        simulation.Solve();

        // Initialise helper variables
        double total_num_edges = 0.0;
        double total_edges_length = 0.0;
        double mismatch_one_num_edges = 0.0;
        double mismatch_one_boundary_length = 0.0;
        double mismatch_two_num_edges = 0.0;
        double mismatch_two_boundary_length = 0.0;

        // Iterate over cells and compute summary statistics
        VertexBasedCellPopulation<2>* p_cell_population = static_cast<VertexBasedCellPopulation<2>*>(&(simulation.rGetCellPopulation()));
        for (VertexBasedCellPopulation<2>::Iterator cell_iter = p_cell_population->Begin();
             cell_iter != p_cell_population->End();
             ++cell_iter)
        {
            // Find this cell's stripe identity
            unsigned cell_stripe_identity = cell_iter->GetCellData()->GetItem("stripe");

            // Get the set of neighbouring element indices
            unsigned elem_index = p_cell_population->GetLocationIndexUsingCell(*cell_iter);
            std::set<unsigned> neighbour_elem_indices = p_cell_population->rGetMesh().GetNeighbouringElementIndices(elem_index);

            // Iterate over these neighbours
            for (std::set<unsigned>::iterator neighbour_iter = neighbour_elem_indices.begin();
                 neighbour_iter != neighbour_elem_indices.end();
                 ++neighbour_iter)
            {
                // Get the length of the edge shared with this neighbour
                unsigned neighbour_index = *neighbour_iter;
                double edge_length = p_cell_population->rGetMesh().GetEdgeLength(elem_index, neighbour_index);

                total_edges_length += edge_length;
                total_num_edges += 1.0;

                // Find this neighbour's stripe identity
                CellPtr p_neighbour = p_cell_population->GetCellUsingLocationIndex(*neighbour_iter);
                unsigned neighbour_stripe_identity = p_neighbour->GetCellData()->GetItem("stripe");

                unsigned num_stripes = 4; ///\todo remove hardcoding
                unsigned mismatch = abs(cell_stripe_identity - neighbour_stripe_identity);
                if (mismatch > num_stripes/2)
                {
                    mismatch = num_stripes - mismatch;
                }

                if (mismatch == 1)
                {
                    mismatch_one_num_edges += 1.0;
                    mismatch_one_boundary_length += edge_length;
                }
                else if (mismatch == 2)
                {
                    mismatch_two_num_edges += 1.0;
                    mismatch_two_boundary_length += edge_length;
                }
                else
                {
                    assert(mismatch == 0);
                }
            }
        }

        // We have counted each cell-cell edge twice
        total_num_edges *= 0.5;
        total_edges_length *= 0.5;
        mismatch_one_num_edges *= 0.5;
        mismatch_one_boundary_length *= 0.5;
        mismatch_two_num_edges *= 0.5;
        mismatch_two_boundary_length *= 0.5;

        // Output summary statistics to results file
        std::cout << heterotypic_line_tension_parameter << "\t"
                        << supercontractile_line_tension_parameter << "\t"
                        << total_num_edges << "\t"
                        << total_edges_length << "\t"
                        << mismatch_one_num_edges << "\t"
                        << mismatch_one_boundary_length
                        << "\t" << mismatch_two_num_edges
                        << "\t" << mismatch_two_boundary_length
                        << "\n";
    }
};

#endif /*TESTSIMULATIONSFORGBEPAPER_HPP_*/
