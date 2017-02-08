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

#ifndef TESTEXAMPLESIMULATION_HPP_
#define TESTEXAMPLESIMULATION_HPP_

#include <cxxtest/TestSuite.h>
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "FakePetscSetup.hpp"
#include "SmartPointers.hpp"
#include "VoronoiVertexMeshGenerator.hpp"
#include "CellsGenerator.hpp"
#include "TransitCellProliferativeType.hpp"
#include "VertexBasedCellPopulation.hpp"
#include "CellLabelWriter.hpp"
#include "OffLatticeSimulation.hpp"
#include "FarhadifarForce.hpp"
#include "RandomGrowthTargetAreaModifier.hpp"
#include "CellAncestor.hpp"
#include "CellAncestorWriter.hpp"
#include "LabelledCloneSizesWriter.hpp"
#include "BiasedVertexBasedDivisionRule.hpp"
#include "CellAgesWriter.hpp"
#include "CellGenerationsWriter.hpp"
#include "AreaBasedCellCycleModel.hpp"
#include "VolumeTrackingModifier.hpp"

class TestExampleSimulation : public AbstractCellBasedWithTimingsTestSuite
{
public:

    void TestExample() throw(Exception)
    {
        EXIT_IF_PARALLEL;

        std::string output_directory = "ExampleSimulation";
        OutputFileHandler results_handler(output_directory, false);
        out_stream overall_results_file = results_handler.OpenOutputFile("overall_results.dat");

        // Set labelling dose, number of labels and waiting time
        double label_probability = 0.2;
        unsigned num_labels = 4;
        double waiting_time = 48.0;

        /*
         * Set max cellular growth rate to 1/6 so that average growth rate is 1/12, hence
         * on average, a cell's target area will grow to twice its initial value
         * in 12 hours.
         */
        double max_growth_rate = 1.0/6.0;

        // Set parameters for initial tissue geometry
        unsigned num_cells_wide = 10;
        unsigned num_cells_high = 10;
        unsigned num_lloyd_steps = 3;
        double target_area = 1.0;

        // Configure simulation output
        double output_timestep = 0.25;

        // Initialise various singletons
        SimulationTime::Destroy();
        SimulationTime::Instance()->SetStartTime(0.0);
        CellPropertyRegistry::Instance()->Clear();
        CellId::ResetMaxCellId();
        RandomNumberGenerator* p_gen = RandomNumberGenerator::Instance();
        p_gen->Reseed(0);

        // Generate random initial mesh
        VoronoiVertexMeshGenerator generator(num_cells_wide, num_cells_high, num_lloyd_steps, target_area);
        MutableVertexMesh<2,2>* p_mesh = generator.GetMesh();

        // Create a vector of cells and associate these with elements of the mesh
        std::vector<CellPtr> cells;
        MAKE_PTR(WildTypeCellMutationState, p_state);
        MAKE_PTR(TransitCellProliferativeType, p_type);
        unsigned num_labelled_cells = 0;
        for (unsigned elem_index=0; elem_index<p_mesh->GetNumElements(); elem_index++)
        {
            AreaBasedCellCycleModel* p_model = new AreaBasedCellCycleModel();
            p_model->SetMaxGeneration(UINT_MAX);

            CellPtr p_cell(new Cell(p_state, p_model));
            p_cell->SetCellProliferativeType(p_type);
            double birth_time = -p_gen->ranf()*12.0;
            p_cell->SetBirthTime(birth_time);

            // With dose-dependent probability, give this cell a random label
            if (p_gen->ranf() < label_probability)
            {
                // Choose one of four labels, uniformly at random
                unsigned label = 1 + p_gen->randMod(num_labels);

                // Label the cell
                p_cell->GetCellData()->SetItem("label", label);
                num_labelled_cells++;

                // Sets the ancestor index of the cell to be its cell ID, to trace its progeny
                MAKE_PTR_ARGS(CellAncestor, p_cell_ancestor, (p_cell->GetCellId()));
                p_cell->SetAncestor(p_cell_ancestor);
            }
            else
            {
                p_cell->GetCellData()->SetItem("label", 0);
            }

            ///\todo Reconsider this bit
            double initial_target_area = 1.0 + p_gen->ranf()*max_growth_rate*(-birth_time);
            p_cell->GetCellData()->SetItem("target area", initial_target_area);

            cells.push_back(p_cell);
        }

        if (num_labelled_cells != 0)
        {
            // Create cell population
            VertexBasedCellPopulation<2> cell_population(*p_mesh, cells);
            cell_population.AddCellWriter<CellLabelWriter>();
            cell_population.AddCellWriter<CellAncestorWriter>();
            cell_population.AddCellWriter<CellAgesWriter>();
            cell_population.AddCellWriter<CellGenerationsWriter>();
            cell_population.SetOutputResultsForChasteVisualizer(false);
            cell_population.SetOutputCellRearrangementLocations(false);

//            // Set cell division rule
//            MAKE_PTR(BiasedVertexBasedDivisionRule<2>, p_div_rule);
//            cell_population.SetVertexBasedDivisionRule(p_div_rule);

            // Create simulation
            OffLatticeSimulation<2> simulation(cell_population);
            simulation.SetOutputDirectory(output_directory);
            simulation.SetSamplingTimestepMultiple(output_timestep/0.002); // Default time step is 0.002 for vertex models
            simulation.SetEndTime(waiting_time);

            // Pass in a force law
            MAKE_PTR(FarhadifarForce<2>, p_force);
            simulation.AddForce(p_force);

            // Pass in a target area modifier
            MAKE_PTR(RandomGrowthTargetAreaModifier<2>, p_growth_modifier);
            p_growth_modifier->SetMaxGrowthRate(max_growth_rate);
            simulation.AddSimulationModifier(p_growth_modifier);

            MAKE_PTR(VolumeTrackingModifier<2>, p_modifier);
            simulation.AddSimulationModifier(p_modifier);

            // Run simulation
            simulation.Solve();

            // Output clone labelling results for this simulation to the overall results file
            std::vector<int> clone_data;

            // Find the 'tissue radius' (largest distance between any vertex to the tissue centroid)
            double tissue_radius = 0.0;
            c_vector<double,2> centroid = simulation.rGetCellPopulation().GetCentroidOfCellPopulation();
            for (AbstractCellPopulation<2,2>::Iterator cell_iter = simulation.rGetCellPopulation().Begin();
                 cell_iter != simulation.rGetCellPopulation().End();
                 ++cell_iter)
            {
                c_vector<double,2> cell_location = simulation.rGetCellPopulation().GetLocationOfCellCentre(*cell_iter);
                double this_distance = norm_2(cell_location - centroid);
                if (this_distance > tissue_radius)
                {
                    tissue_radius = this_distance;
                }
            }

            /*
             * Assuming that the tissue shape is 'not too' anisotropic, store a set of cells whose
             * centroids lie within a radius 0.7 times the tissue radius. We will only consider
             * those labelled clones that contain cells within this region of the tissue when
             * performing clonal analysis. This is intended to reduce edge effects in our analysis.
             */
            double scaling = 0.7;
            std::set<CellPtr> cells_to_cover;
            for (AbstractCellPopulation<2,2>::Iterator cell_iter = simulation.rGetCellPopulation().Begin();
                 cell_iter != simulation.rGetCellPopulation().End();
                 ++cell_iter)
            {
                c_vector<double,2> cell_location = simulation.rGetCellPopulation().GetLocationOfCellCentre(*cell_iter);
                if (norm_2(cell_location - centroid) < scaling*tissue_radius)
                {
                    cells_to_cover.insert(*cell_iter);
                }
            }

            std::set<unsigned> covered_cells;
            for (std::set<CellPtr>::iterator cell_iter = cells_to_cover.begin();
                 cell_iter != cells_to_cover.end();
                 ++cell_iter)
            {
                unsigned location_index = simulation.rGetCellPopulation().GetLocationIndexUsingCell(*cell_iter);
                if (covered_cells.find(location_index) == covered_cells.end())
                {
                    covered_cells.insert(location_index);

                    unsigned cell_label = (*cell_iter)->GetCellData()->GetItem("label");

                    std::set<unsigned> ancestors_this_clone;
                    ancestors_this_clone.insert((*cell_iter)->GetAncestor());

                    if (cell_label != 0)
                    {
                        std::set<unsigned> cells_in_labelled_clone;
                        cells_in_labelled_clone.insert(location_index);

                        std::set<unsigned> neighbours_to_add = simulation.rGetCellPopulation().GetNeighbouringLocationIndices(*cell_iter);
                        for (std::set<unsigned>::iterator iter = neighbours_to_add.begin(); iter != neighbours_to_add.end();)
                        {
                           if (covered_cells.find(*iter) != covered_cells.end())
                           {
                               neighbours_to_add.erase(iter++);
                           }
                           else
                           {
                               CellPtr p_neighbour = simulation.rGetCellPopulation().GetCellUsingLocationIndex(*iter);
                               unsigned neighbour_label = p_neighbour->GetCellData()->GetItem("label");
                               if (neighbour_label != cell_label)
                               {
                                   neighbours_to_add.erase(iter++);
                               }
                               else
                               {
                                   covered_cells.insert(*iter);
                                   ++iter;
                               }
                           }
                        }

                        for (std::set<unsigned>::iterator iter = neighbours_to_add.begin(); iter != neighbours_to_add.end(); ++iter)
                        {
                            cells_in_labelled_clone.insert(*iter);
                            ancestors_this_clone.insert(simulation.rGetCellPopulation().GetCellUsingLocationIndex(*iter)->GetAncestor());
                        }

                        std::set<unsigned> neighbours_to_check = neighbours_to_add;
                        std::set<unsigned>::iterator neighbour_iter = neighbours_to_check.begin();
                        while (!neighbours_to_check.empty())
                        {
                            CellPtr p_neighbour = simulation.rGetCellPopulation().GetCellUsingLocationIndex(*neighbour_iter);
                            std::set<unsigned> other_neighbours_to_add = simulation.rGetCellPopulation().GetNeighbouringLocationIndices(p_neighbour);
                            for (std::set<unsigned>::iterator other_iter = other_neighbours_to_add.begin(); other_iter != other_neighbours_to_add.end();)
                            {
                               if (covered_cells.find(*other_iter) != covered_cells.end())
                               {
                                   other_neighbours_to_add.erase(other_iter++);
                               }
                               else
                               {
                                   CellPtr p_other_neighbour = simulation.rGetCellPopulation().GetCellUsingLocationIndex(*other_iter);
                                   unsigned neighbour_label = p_other_neighbour->GetCellData()->GetItem("label");
                                   if (neighbour_label != cell_label)
                                   {
                                       other_neighbours_to_add.erase(other_iter++);
                                   }
                                   else
                                   {
                                       covered_cells.insert(*other_iter);
                                       ++other_iter;
                                   }
                               }
                            }

                            for (std::set<unsigned>::iterator other_iter = other_neighbours_to_add.begin(); other_iter != other_neighbours_to_add.end(); ++other_iter)
                            {
                                cells_in_labelled_clone.insert(*other_iter);
                                neighbours_to_check.insert(*other_iter);
                            }

                            neighbours_to_check.erase(*neighbour_iter++);
                        }

                        clone_data.push_back(cells_in_labelled_clone.size());
                        for (std::set<unsigned>::iterator it = ancestors_this_clone.begin(); it != ancestors_this_clone.end(); ++it)
                        {
                            clone_data.push_back(*it);
                        }
                        clone_data.push_back(-1);
                    }
                }
            }

            for (unsigned i=0; i<clone_data.size(); i++)
            {
                *overall_results_file << clone_data[i] << "\t" << std::flush;
            }
            *overall_results_file << "\n" << std::flush;
        }

        // Close results files and tidy up
        overall_results_file->close();
    }
};

#endif /* TESTEXAMPLESIMULATION_HPP_ */
