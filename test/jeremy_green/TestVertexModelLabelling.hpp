
#ifndef TESTVERTEXMODELLABELLING_HPP_
#define TESTVERTEXMODELLABELLING_HPP_

#include <cxxtest/TestSuite.h>
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "FakePetscSetup.hpp"
#include "SmartPointers.hpp"
#include "VoronoiVertexMeshGenerator.hpp"
#include "CellsGenerator.hpp"
#include "TransitCellProliferativeType.hpp"
#include "VertexBasedCellPopulation.hpp"
#include "OffLatticeSimulation.hpp"
#include "FarhadifarForce.hpp"
#include "CellAncestor.hpp"
#include "UniformCellCycleModel.hpp"
#include "ConstantTargetAreaModifier.hpp"
#include "ClusterDataWriter.hpp"

class TestVertexModelLabelling : public AbstractCellBasedWithTimingsTestSuite
{
public:

    void TestLabelling() throw(Exception)
    {
        ///\todo Consider different labelling proportions
        double labelling_proportion[1] = {0.2};

        // Set number of labels and waiting time
        unsigned num_labels = 4;
        double max_waiting_time = 30.0;

        // Set lower and upper bounds on cell cycle times (in hours)
        double min_cycle_time = 10;
        double max_cycle_time = 14;

        // Set parameters for initial tissue geometry
        unsigned num_cells_wide = 17;
        unsigned num_cells_high = 17;
        unsigned num_lloyd_steps = 3;

        unsigned num_sims_per_labelling_prop = 1;

        for (unsigned prob_index=0; prob_index<1; prob_index++)
        {
            // Set labelling dose
            double proportion_of_cells_to_label = labelling_proportion[prob_index];

            for (unsigned sim_index=0; sim_index<num_sims_per_labelling_prop; sim_index++)
            {
                std::stringstream out;
                out << "VertexModelLabelling/Labels4/Prop" << proportion_of_cells_to_label << "/Sim" << sim_index;
                std::string output_directory = out.str();
                OutputFileHandler results_handler(output_directory, false);

                // Initialise various singletons
                SimulationTime::Destroy();
                SimulationTime::Instance()->SetStartTime(0.0);
                CellPropertyRegistry::Instance()->Clear();
                CellId::ResetMaxCellId();
                RandomNumberGenerator* p_gen = RandomNumberGenerator::Instance();
                p_gen->Reseed(sim_index);

                // Generate random initial mesh
                VoronoiVertexMeshGenerator generator(num_cells_wide, num_cells_high, num_lloyd_steps);
                MutableVertexMesh<2,2>* p_mesh = generator.GetMesh();
                unsigned num_cells = p_mesh->GetNumElements();

                // Create a vector of cells and associate these with elements of the mesh
                std::vector<CellPtr> cells;
                MAKE_PTR(WildTypeCellMutationState, p_state);
                MAKE_PTR(TransitCellProliferativeType, p_type);
                for (unsigned elem_index=0; elem_index<num_cells; elem_index++)
                {
                    UniformCellCycleModel* p_model = new UniformCellCycleModel();
                    p_model->SetDimension(2);
                    p_model->SetMinCellCycleDuration(min_cycle_time);
                    p_model->SetMaxCellCycleDuration(max_cycle_time);

                    CellPtr p_cell(new Cell(p_state, p_model));
                    p_cell->SetCellProliferativeType(p_type);
                    p_cell->SetBirthTime(-p_gen->ranf()*8.0);
                    p_cell->GetCellData()->SetItem("label", 0);

                    cells.push_back(p_cell);
                }

                // Randomly label the correct number of cells, choosing a random colour in each case
                unsigned num_cells_to_label = proportion_of_cells_to_label*num_cells;
                unsigned labelling_index = 0;
                unsigned num_cells_labelled_so_far = 0;
                double u;
                while (num_cells_labelled_so_far < num_cells_to_label)
                {
                    u = p_gen->ranf();
                    double r = (num_cells - labelling_index)*u;
                    if (r < num_cells_to_label - num_cells_labelled_so_far)
                    {
                        // Choose a random colour and label the cell
                        unsigned label = 1 + p_gen->randMod(num_labels);
                        cells[labelling_index]->GetCellData()->SetItem("label", label);
                        num_cells_labelled_so_far++;
                    }
                    labelling_index++;
                }

                // Create cell population
                VertexBasedCellPopulation<2> cell_population(*p_mesh, cells);
                cell_population.SetOutputResultsForChasteVisualizer(false);
                cell_population.SetOutputCellRearrangementLocations(false);
                cell_population.SetCellAncestorsToLocationIndices();
                cell_population.AddPopulationWriter<ClusterDataWriter>();

                // Create simulation
                OffLatticeSimulation<2> simulation(cell_population);
                simulation.SetOutputDirectory(output_directory);
                simulation.SetSamplingTimestepMultiple(1.0/0.002); // Default time step is 0.002 for vertex models
                simulation.SetEndTime(max_waiting_time);

                // Pass in a force law
                MAKE_PTR(FarhadifarForce<2>, p_force);
                simulation.AddForce(p_force);

                // Pass in a target area modifier
                MAKE_PTR(ConstantTargetAreaModifier<2>, p_modifier);
                simulation.AddSimulationModifier(p_modifier);

                // Run simulation
                simulation.Solve();
            }
        }
    }
};

#endif /* TESTVERTEXMODELLABELLING_HPP_ */
