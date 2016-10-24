
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
#include "SmartPointers.hpp"
#include "FakePetscSetup.hpp"

class TestMyosin : public AbstractCellBasedWithTimingsTestSuite
{
public:

    void TestSimulation() throw (Exception)
    {
        // Specify tissue geometry
        unsigned num_cells_wide = 14;
        unsigned num_cells_high = 6;

        // Create a mesh
        HoneycombMeshGenerator generator(num_cells_wide, num_cells_high);
        MutableMesh<2,2>* p_mesh = generator.GetMesh();

        // Create some non-proliferating cells
        std::vector<CellPtr> cells;
        CellsGenerator<NoCellCycleModel, 2> cells_generator;
        cells_generator.GenerateBasic(cells, p_mesh->GetNumNodes());

        // Specify mechanical parameters
        ///\todo

        // Set max simulation time
        double max_simulation_time = 100.0;

        // Specify time steps
        double time_step = 0.01;
        double output_time_step = 1.0;

        // Create a cell population that associates the cells with the mesh
        MeshBasedCellPopulation<2> cell_population(*p_mesh, cells);
        cell_population.SetOutputResultsForChasteVisualizer(false);

        // Create a simulation using the cell population
        OffLatticeSimulation<2> simulation(cell_population);
        simulation.SetOutputDirectory("TestMyosin");
        simulation.SetEndTime(max_simulation_time);

        simulation.SetDt(time_step);
        unsigned output_time_step_multiple = (unsigned) (output_time_step/time_step);
        simulation.SetSamplingTimestepMultiple(output_time_step_multiple);

        // Create a force law
        ///\todo replace with weighted springs
        MAKE_PTR(GeneralisedLinearSpringForce<2>, p_force);
        simulator.AddForce(p_force);

        simulation.AddForce(p_force);

        // Run simulation
        simulation.Solve();
    }
};

#endif /*TESTMYOSIN_HPP_*/
