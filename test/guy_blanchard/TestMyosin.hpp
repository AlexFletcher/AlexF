
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
#include "Debug.hpp"

class TestMyosin : public AbstractCellBasedWithTimingsTestSuite
{
public:

    void TestSimulation() throw (Exception)
    {
        // Specify proportion of cells that have myosin
        double myosin_proportion = 0.7;

        // Specify mechanical parameters
        double myosin_spring_stiffness = 2.0;
        double myosin_spring_natural_length = 0.5;
        double non_myosin_spring_stiffness = 1.0;
        double non_myosin_spring_natural_length = 1.0;

        // Specify tissue geometry
        unsigned num_cells_wide = 14;
        unsigned num_cells_high = 6;

        // Create a mesh
        HoneycombMeshGenerator generator(num_cells_wide, num_cells_high);
        MutableMesh<2,2>* p_mesh = generator.GetMesh();

        for (unsigned i=0; i<p_mesh->GetNumNodes(); i++)
        {
            PRINT_VARIABLE(p_mesh->GetNode(i)->GetIndex());
            PRINT_VARIABLE(p_mesh->GetNode(i)->IsBoundaryNode());
        }

        // Record the boundaries of this mesh for the sliding boundary conditions that are imposed below
        ChasteCuboid<2> bounds = p_mesh->CalculateBoundingBox();
        double x_min = bounds.rGetLowerCorner()[0];
        double y_min = bounds.rGetLowerCorner()[1];
        double x_max = bounds.rGetUpperCorner()[0];
        double y_max = bounds.rGetUpperCorner()[1];

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

        // Set max simulation time
        double max_simulation_time = 100.0; ///\todo simulate to equilibrium

        // Specify time steps
        double time_step = 0.01;
        double output_time_step = 1.0;

        // Create a cell population that associates the cells with the mesh
        MeshBasedCellPopulationWithoutRemeshing<2> cell_population(*p_mesh, cells);
        cell_population.SetOutputResultsForChasteVisualizer(true);

        // Create a simulation using the cell population
        OffLatticeSimulation<2> simulation(cell_population);
        simulation.SetOutputDirectory("TestMyosin");
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

        // Create some sliding boundary conditions and pass them to the simulation
        ///\todo just replace with PlaneBoundaryCondition?

        // Impose a sliding condition at the left boundary
        c_vector<double,2> point = zero_vector<double>(2);
        point(0) = x_min;
        point(1) = y_min;
        c_vector<double,2> normal = zero_vector<double>(2);
        normal(0) = -1.0;
        MAKE_PTR_ARGS(SlidingBoundaryCondition<2>, p_bc_left, (&cell_population, point, normal));
        simulation.AddCellPopulationBoundaryCondition(p_bc_left);

        // Impose a sliding condition at the right boundary
        point(0) = x_max;
        normal(0) = 1.0;
        MAKE_PTR_ARGS(SlidingBoundaryCondition<2>, p_bc_right, (&cell_population, point, normal));
        simulation.AddCellPopulationBoundaryCondition(p_bc_right);

        // Impose a sliding condition at the bottom boundary
        point(0) = x_max;
        normal(0) = 0.0;
        normal(1) = -1.0;
        MAKE_PTR_ARGS(SlidingBoundaryCondition<2>, p_bc_bottom, (&cell_population, point, normal));
        simulation.AddCellPopulationBoundaryCondition(p_bc_bottom);

        // Impose a sliding condition at the top boundary
        point(1) = y_max;
        normal(1) = 1.0;
        MAKE_PTR_ARGS(SlidingBoundaryCondition<2>, p_bc_top, (&cell_population, point, normal));
        simulation.AddCellPopulationBoundaryCondition(p_bc_top);

        // Run simulation
        simulation.Solve();
    }
};

#endif /*TESTMYOSIN_HPP_*/
