
#ifndef TESTMYOSIN_HPP_
#define TESTMYOSIN_HPP_

#include <cxxtest/TestSuite.h>
#include "CheckpointArchiveTypes.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "HoneycombMeshGenerator.hpp"
#include "CellsGenerator.hpp"
#include "NoCellCycleModel.hpp"
#include "MeshBasedCellPopulationWithoutRemeshing.hpp"
#include "OffLatticeSimulation.hpp"
#include "MyosinWeightedSpringForce.hpp"
#include "SlidingBoundaryCondition.hpp"
#include "SmartPointers.hpp"
#include "FakePetscSetup.hpp"
#include "CellLabel.hpp"
#include "RandomNumberGenerator.hpp"

class TestMyosin : public AbstractCellBasedWithTimingsTestSuite
{
public:

    void TestSimulation() throw (Exception)
    {
        // Specify mechanical parameters
        double myosin_spring_stiffness = 100.0;
        double myosin_spring_natural_length = 0.5;
        double non_myosin_spring_stiffness = 1.0;
        double non_myosin_spring_natural_length = 1.0;

        // Specify tissue geometry
        unsigned num_cells_wide = 14;
        unsigned num_cells_high = 6;

        unsigned num_simulations = 50;

        // Set max simulation time
        double max_simulation_time = 20.0; ///\todo simulate to equilibrium

        // Specify time steps
        double time_step = 0.0005;
        double output_time_step = 50.0;

        // Create overall results file
        std::string output_directory = "TestMyosin";
        OutputFileHandler results_handler(output_directory, false);
        out_stream results_file = results_handler.OpenOutputFile("summary_statistic.dat");

        for (unsigned myosin_index=0; myosin_index<20; myosin_index++)
        {
            // Specify proportion of cells that have myosin
            double myosin_proportion = ((double) myosin_index + 1)/20.0;

            *results_file << myosin_proportion << "\t" << std::flush;
            for (unsigned sim_index=0; sim_index<num_simulations; sim_index++)
            {
                // Initialise various singletons
                SimulationTime::Destroy();
                SimulationTime::Instance()->SetStartTime(0.0);
                CellPropertyRegistry::Instance()->Clear();
                CellId::ResetMaxCellId();
                RandomNumberGenerator* p_gen = RandomNumberGenerator::Instance();
                p_gen->Reseed(sim_index + num_simulations*myosin_index);

                // Create a mesh
                HoneycombMeshGenerator generator(num_cells_wide, num_cells_high);
                MutableMesh<2,2>* p_mesh = generator.GetMesh();

                // Record the boundaries of this mesh for the sliding boundary conditions that are imposed below
                ChasteCuboid<2> cuboid = p_mesh->CalculateBoundingBox();
                double x_min = cuboid.rGetLowerCorner()[0];
                double y_min = cuboid.rGetLowerCorner()[1];
                double x_max = cuboid.rGetUpperCorner()[0];
                double y_max = cuboid.rGetUpperCorner()[1];

                // Create some non-proliferating cells
                std::vector<CellPtr> cells;
                CellsGenerator<NoCellCycleModel, 2> cells_generator;
                cells_generator.GenerateBasic(cells, p_mesh->GetNumNodes());

                // Label the correct proportion of cells as having myosin and randomise their order
                unsigned num_cells = cells.size();
                unsigned num_labelled_cells = myosin_proportion*num_cells;
                for (unsigned i=0; i<num_labelled_cells; i++)
                {
                    cells[i]->AddCellProperty(CellPropertyRegistry::Instance()->Get<CellLabel>());
                }
                for (unsigned end=num_cells-1; end>0; end--)
                {
                    unsigned k = RandomNumberGenerator::Instance()->randMod(end+1);
                    CellPtr p_temp_cell = cells[end];
                    cells[end] = cells[k];
                    cells[k] = p_temp_cell;
                }

                // Create a cell population that associates the cells with the mesh
                MeshBasedCellPopulationWithoutRemeshing<2> cell_population(*p_mesh, cells);
                cell_population.SetOutputResultsForChasteVisualizer(true);

                // Create a simulation using the cell population
                OffLatticeSimulation<2> simulation(cell_population);
                simulation.SetOutputDirectory(output_directory);
                simulation.SetEndTime(max_simulation_time);
                simulation.SetDt(time_step);
                unsigned output_time_step_multiple = (unsigned) (output_time_step/time_step);
                simulation.SetSamplingTimestepMultiple(output_time_step_multiple);

                // Create a force law
                MAKE_PTR(MyosinWeightedSpringForce<2>, p_force);
                p_force->SetMyosinSpringStiffness(myosin_spring_stiffness);
                p_force->SetMyosinSpringNaturalLength(myosin_spring_natural_length);
                p_force->SetNonMyosinSpringStiffness(non_myosin_spring_stiffness);
                p_force->SetNonMyosinSpringNaturalLength(non_myosin_spring_natural_length);
                simulation.AddForce(p_force);

                // Impose a sliding condition at each boundary
                MAKE_PTR_ARGS(SlidingBoundaryCondition, p_bc, (&cell_population, x_min, y_min, x_max, y_max));
                simulation.AddCellPopulationBoundaryCondition(p_bc);

                // Run simulation
                simulation.Solve();

                // Generate summary statistic
                double average_force = 0.0;
                for (unsigned row_index=0; row_index<num_cells_high; row_index++)
                {
                    double average_force_this_row = 0.0;
                    for (unsigned column_index=0; column_index<num_cells_wide-1; column_index++)
                    {
                        unsigned node_index = row_index*num_cells_wide + column_index;

                        c_vector<double,2> spring_force = p_force->CalculateForceBetweenNodes(node_index, node_index+1, simulation.rGetCellPopulation());

                        c_vector<double,2> loc1 = simulation.rGetCellPopulation().GetNode(node_index)->rGetLocation();
                        c_vector<double,2> loc2 = simulation.rGetCellPopulation().GetNode(node_index+1)->rGetLocation();

                        c_vector<double, 2> unit_vec = loc2 - loc1;
                        double vec_length = norm_2(unit_vec);
                        unit_vec /= vec_length;

                        average_force_this_row += (spring_force[0]*unit_vec[0] + spring_force[1]*unit_vec[1])/((double)num_cells_wide-1.0);
                    }
                    average_force += average_force_this_row/((double) num_cells_high);
                }
                    *results_file << average_force << "\t" << std::flush;
            }
            *results_file << "\n" << std::flush;
        }

        // Close results files and tidy up
        results_file->close();
    }
};

#endif /*TESTMYOSIN_HPP_*/
