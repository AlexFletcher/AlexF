#ifndef TESTMUKULSIMULATION_HPP_
#define TESTMUKULSIMULATION_HPP_

#include <cxxtest/TestSuite.h>
#include "SmartPointers.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "HoneycombMeshGenerator.hpp"
#include "CellsGenerator.hpp"
#include "DefaultCellProliferativeType.hpp"
#include "NoCellCycleModel.hpp"
#include "NodeBasedCellPopulation.hpp"
#include "CellDataItemWriter.hpp"
#include "RepulsionForce.hpp"
#include "OffLatticeSimulation.hpp"
#include "CircularBoundaryCondition.hpp"
#include "MukulPdeSystemSolver.hpp"
#include "MukulPdeSystem.hpp"
#include "PetscSetupAndFinalize.hpp"

class TestMukulSimulation : public AbstractCellBasedWithTimingsTestSuite
{
public:

    void TestModel() throw (Exception)
    {
        RandomNumberGenerator* p_gen = RandomNumberGenerator::Instance();

        // Specify radius of disk domain
        double domain_radius = 10.0;

        // Create a pointer to a mesh object
        HoneycombMeshGenerator generator(2.5*domain_radius, 2.5*domain_radius);
        MutableMesh<2,2>* p_gen_mesh = generator.GetCircularMesh(domain_radius);
        NodesOnlyMesh<2>* p_mesh = new NodesOnlyMesh<2>;
        double cut_off = 1.5;
        p_mesh->ConstructNodesWithoutMesh(*p_gen_mesh, cut_off);

        // Create a vector of cell objects
        std::vector<CellPtr> cells;
	    MAKE_PTR(WildTypeCellMutationState, p_state);
        MAKE_PTR(DefaultCellProliferativeType, p_type);
        for (unsigned i=0; i<p_mesh->GetNumNodes(); i++)
        {
            ///\todo replace with suitable cell cycle model
            NoCellCycleModel* p_cycle_model = new NoCellCycleModel();
            p_cycle_model->SetDimension(2);

            CellPtr p_cell(new Cell(p_state, p_cycle_model));
            p_cell->SetCellProliferativeType(p_type);
            p_cell->SetBirthTime(-p_gen->ranf()); ///\todo replace with suitable initial age
            p_cell->InitialiseCellCycleModel();

            p_cell->GetCellData()->SetItem("bmp", 1.0); ///\todo replace with suitable initial condition
            p_cell->GetCellData()->SetItem("nog", 1.0); ///\todo replace with suitable initial condition

            cells.push_back(p_cell);
        }

        // Create a cell population object
        NodeBasedCellPopulation<2> cell_population(*p_mesh, cells);
        c_vector<double,2> pop_centre = cell_population.GetCentroidOfCellPopulation();

        ///\todo Specify what data to output to results files

        // Make a cell data writer so we can pass in a variable name
        boost::shared_ptr<CellDataItemWriter<2,2> > p_cell_data_item_writer(new CellDataItemWriter<2,2>("bmp")); ///\todo amend as appropriate
        cell_population.AddCellWriter(p_cell_data_item_writer);

        // Create a cell-based simulation object
        OffLatticeSimulation<2> simulator(cell_population);
        simulator.SetOutputDirectory("Mukul");
        simulator.SetSamplingTimestepMultiple(120);
        simulator.SetEndTime(2.0);

        // Create a force law and pass it to the simulation
        MAKE_PTR(RepulsionForce<2>, p_force); ///\todo replace with spring-like force and/or random motion
        simulator.AddForce(p_force);

        // Create finite element mesh for PDE system
        TrianglesMeshReader<2,2> mesh_reader("mesh/test/data/disk_984_elements");
        MutableMesh<2,2> fe_mesh;
        fe_mesh.ConstructFromMeshReader(mesh_reader);
        fe_mesh.Translate(-pop_centre);
        fe_mesh.Scale(domain_radius, domain_radius);

        // Create PDE system object
        MAKE_PTR(MukulPdeSystem<2>, p_pde_system);

        // Create non-zero Dirichlet boundary conditions for each state variable
        ///\todo can change to no-flux for one of them
        BoundaryConditionsContainer<2,2,2> bcc;
        ConstBoundaryCondition<2>* p_bc_for_u = new ConstBoundaryCondition<2>(2.0);
        ConstBoundaryCondition<2>* p_bc_for_v = new ConstBoundaryCondition<2>(0.75);
        for (TetrahedralMesh<2,2>::BoundaryNodeIterator iter = fe_mesh.GetBoundaryNodeIteratorBegin();
             iter != fe_mesh.GetBoundaryNodeIteratorEnd();
             iter++)
        {
            ///\todo Is this the correct way to specify BCs for two state variables?
            bcc.AddDirichletBoundaryCondition(*iter, p_bc_for_u);
            bcc.AddDirichletBoundaryCondition(*iter, p_bc_for_v);
        }

        // Create a bespoke solver for the PDE system and pass it to the simulation
        MAKE_PTR_ARGS(MukulPdeSystemSolver<2>, p_solver, (p_pde_system, &bcc, &fe_mesh));
        simulator.AddSimulationModifier(p_solver);

        // Create a boundary condition object to prevent cells moving outside a specified disk
        MAKE_PTR_ARGS(CircularBoundaryCondition, p_disk_condition, (&cell_population, pop_centre, domain_radius));
        simulator.AddCellPopulationBoundaryCondition(p_disk_condition);

        // Run the simulation
        simulator.Solve();

        // Avoid memory leaks
        delete p_mesh;
    }
};

#endif /* TESTMUKULSIMULATION_HPP_ */
