
#ifndef TESTNODEBASEDGUT_HPP_
#define TESTNODEBASEDGUT_HPP_

#include <cxxtest/TestSuite.h>
#include "AbstractCellBasedTestSuite.hpp"
#include "CheckpointArchiveTypes.hpp"
#include "SmartPointers.hpp"
#include "HoneycombMeshGenerator.hpp"
#include "Cylindrical2dNodesOnlyMesh.hpp"
#include "CellsGenerator.hpp"
#include "TransitCellProliferativeType.hpp"
#include "UniformCellCycleModel.hpp"
#include "NodeBasedCellPopulation.hpp"
#include "RandomDirectionCentreBasedDivisionRule.hpp"
#include "OffLatticeSimulation.hpp"
#include "RepulsionForce.hpp"
#include "FakePetscSetup.hpp"

class TestNodeBasedGut : public AbstractCellBasedTestSuite
{
public:

    void TestSimulation() throw(Exception)
    {
        // Set cutoff for cell-cell mechanical interactions
        double max_cell_interaction_radius = 1.5;

        // Specify minimum and maxmimum cell-cycle duration (in hours)
        // Note: in this simulation, each cell cycle time is drawn independently from U[min_cycle_duration,max_cycle_duration]
        double min_cycle_duration = 12.0;
        double max_cycle_duration = 18.0;

        // Specify initial conditions and simulation duration (in hours)
        unsigned initial_num_cells_across = 4;
        unsigned initial_num_cells_up = 30;
        double end_time = 72.0;

        // The default timestep is 1/120 hours; the output timestep is this multipled by the value below
        double sampling_multiple = 10;

        // Create a mesh object
        HoneycombMeshGenerator generator(initial_num_cells_across, initial_num_cells_up);
        MutableMesh<2,2>* p_generating_mesh = generator.GetMesh();
        NodesOnlyMesh<2> mesh;
        mesh.ConstructNodesWithoutMesh(*p_generating_mesh, max_cell_interaction_radius);

        // Create cell objects, one per node
        std::vector<CellPtr> cells;
        MAKE_PTR(TransitCellProliferativeType, p_type);
        CellsGenerator<UniformCellCycleModel, 2> cells_generator;
        cells_generator.GenerateBasicRandom(cells, mesh.GetNumNodes(), p_type);
        for (unsigned i=0; i<cells.size(); i++)
        {
            static_cast<UniformCellCycleModel*>(cells[i]->GetCellCycleModel())->SetMinCellCycleDuration(min_cycle_duration);
            static_cast<UniformCellCycleModel*>(cells[i]->GetCellCycleModel())->SetMaxCellCycleDuration(max_cycle_duration);
        }

        // Create a cell population object
        NodeBasedCellPopulation<2> cell_population(mesh, cells);

        // Specify a cell division orientation rule
        typedef RandomDirectionCentreBasedDivisionRule<2,2> FixedRule;
        MAKE_PTR(FixedRule, p_div_rule);
        cell_population.SetCentreBasedDivisionRule(p_div_rule);

        // Create a simulation object and specify the output directory
        OffLatticeSimulation<2> simulator(cell_population);
        simulator.SetEndTime(end_time);
        simulator.SetSamplingTimestepMultiple(sampling_multiple);
        simulator.SetOutputDirectory("TestNodeBasedGut");

        // Create a simple two-body repulsion force law for pairs of neighbouring cells
        MAKE_PTR(RepulsionForce<2>, p_force);
        simulator.AddForce(p_force);

        // Call Solve() to run the simulation
        simulator.Solve();
    }


    void TestCylindricalSimulation() throw(Exception)
    {
        // Set cutoff for cell-cell mechanical interactions
        double max_cell_interaction_radius = 1.5;

        // Specify (fixed) circumference of cylinder ////
        double periodic_width = 6.0; ////

        // Specify minimum and maxmimum cell-cycle duration (in hours)
        // Note: in this simulation, each cell cycle time is drawn independently from U[min_cycle_duration,max_cycle_duration]
        double min_cycle_duration = 12.0;
        double max_cycle_duration = 18.0;

        // Specify initial conditions and simulation duration (in hours)
        unsigned initial_num_cells_across = 4;
        unsigned initial_num_cells_up = 30;
        double end_time = 72.0;

        // The default timestep is 1/120 hours; the output timestep is this multipled by the value below
        double sampling_multiple = 10;

        // Create a cylindrical mesh object
        HoneycombMeshGenerator generator(initial_num_cells_across, initial_num_cells_up);
        MutableMesh<2,2>* p_generating_mesh = generator.GetMesh();
        Cylindrical2dNodesOnlyMesh mesh(periodic_width); ////
        mesh.ConstructNodesWithoutMesh(*p_generating_mesh, max_cell_interaction_radius); ////

        // Create cell objects, one per node
        std::vector<CellPtr> cells;
        MAKE_PTR(TransitCellProliferativeType, p_type);
        CellsGenerator<UniformCellCycleModel, 2> cells_generator;
        cells_generator.GenerateBasicRandom(cells, mesh.GetNumNodes(), p_type);
        for (unsigned i=0; i<cells.size(); i++)
        {
            static_cast<UniformCellCycleModel*>(cells[i]->GetCellCycleModel())->SetMinCellCycleDuration(min_cycle_duration);
            static_cast<UniformCellCycleModel*>(cells[i]->GetCellCycleModel())->SetMaxCellCycleDuration(max_cycle_duration);
        }

        // Create a cell population object
        NodeBasedCellPopulation<2> cell_population(mesh, cells);

        // Specify a cell division orientation rule
        typedef RandomDirectionCentreBasedDivisionRule<2,2> FixedRule;
        MAKE_PTR(FixedRule, p_div_rule);
        cell_population.SetCentreBasedDivisionRule(p_div_rule);

        // Create a simulation object and specify the output directory
        OffLatticeSimulation<2> simulator(cell_population);
        simulator.SetEndTime(end_time);
        simulator.SetSamplingTimestepMultiple(sampling_multiple);
        simulator.SetOutputDirectory("TestCylindricalNodeBasedGut");

        // Create a simple two-body repulsion force law for pairs of neighbouring cells
        MAKE_PTR(RepulsionForce<2>, p_force);
        simulator.AddForce(p_force);

        // Call Solve() to run the simulation
        simulator.Solve();
    }
};

#endif /* TESTNODEBASEDGUT_HPP_ */
