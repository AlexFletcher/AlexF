#ifndef TESTMUKULSIMULATION_HPP_
#define TESTMUKULSIMULATION_HPP_

// The first thing to do is to include the necessary header files

#include <cxxtest/TestSuite.h>
#include "SmartPointers.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "CellsGenerator.hpp"
#include "DefaultCellProliferativeType.hpp"
#include "MorphogenDependentCellCycleModel.hpp" ///\todo replace with suitable cell cycle model
#include "NodeBasedCellPopulation.hpp"
#include "RepulsionForce.hpp" ///\todo replace with spring-like force and/or random motion?
#include "OffLatticeSimulation.hpp"
#include "ParabolicGrowingDomainPdeModifier.hpp" ///\todo replace with fixed (disk) domain solver
#include "MorphogenCellwiseSourceParabolicPde.hpp" ///\todo replace with suitable PDE(s)
#include "PetscSetupAndFinalize.hpp"

class TestMukulSimulation : public AbstractCellBasedWithTimingsTestSuite
{
public:

    void TestNodeBasedMorphogen() throw (Exception)
    {
        RandomNumberGenerator* p_gen = RandomNumberGenerator::Instance();

        // Specify radius of disk domain
        double domain_radius = 10.0;

        // Create a pointer to a 'nodes only' mesh object
        HoneycombMeshGenerator generator(1.25*domain_radius, 1.25*domain_radius);
        MutableMesh<2,2>* p_generating_mesh = generator.GetCircularMesh();
        NodesOnlyMesh<2>* p_mesh = new NodesOnlyMesh<2>;
        double cut_off_length = 1.5;
        p_mesh->ConstructNodesWithoutMesh(*p_generating_mesh, cut_off_length);

        // Create a vector of cell objects
        std::vector<CellPtr> cells;
	    MAKE_PTR(WildTypeCellMutationState, p_state);
        MAKE_PTR(DefaultCellProliferativeType, p_type);
        for (unsigned i=0; i<num_cells; i++)
        {
            MorphogenDependentCellCycleModel* p_cycle_model = new MorphogenDependentCellCycleModel(); ///\todo replace with suitable cell cycle model
            p_cycle_model->SetDimension(2);
            p_cycle_model->SetCurrentMass(0.5*(p_gen->ranf()+1.0));
            p_cycle_model->SetMorphogenInfluence(10.0);

            CellPtr p_cell(new Cell(p_state, p_cycle_model));
            p_cell->SetCellProliferativeType(p_type);
            p_cell->SetBirthTime(-p_gen->ranf()); ///\todo replace with suitable initial age
            p_cell->InitialiseCellCycleModel();
            p_cell->GetCellData()->SetItem("morphogen", 0.0); ///\todo replace with suitable initial condition

            cells.push_back(p_cell);
        }

        // Create a cell population object
        NodeBasedCellPopulation<2> cell_population(*p_mesh, cells);

        ///\todo Specify what data to output to results files

        // Make a cell data writer so we can pass in a variable name
        boost::shared_ptr<CellDataItemWriter<2,2> > p_cell_data_item_writer(new CellDataItemWriter<2,2>("morphogen")); ///\todo amend as appropriate
        cell_population.AddCellWriter(p_cell_data_item_writer);

        // Create a cell-based simulation object
        OffLatticeSimulation<2> simulator(cell_population);
        simulator.SetOutputDirectory("Mukule");
        simulator.SetDt(1.0/200.0);
        simulator.SetSamplingTimestepMultiple(10);
        simulator.SetEndTime(10.0);

        // Create a force law and pass it to the simulation
        MAKE_PTR(RepulsionForce<2>, p_force);
        simulator.AddForce(p_force);

        // Create PDE and boundary condition objects
        MorphogenCellwiseSourceParabolicPde<2> pde(cell_population, 1.0, 1.0, 0.01); ///\todo amend as appropriate
        ConstBoundaryCondition<2> bc(0.0);  ///\todo amend as appropriate
        ParabolicPdeAndBoundaryConditions<2> pde_and_bc(&pde, &bc, true);
        pde_and_bc.SetDependentVariableName("morphogen");

        // Create a PDE modifier object and pass to the simulation
        MAKE_PTR_ARGS(ParabolicGrowingDomainPdeModifier<2>, p_pde_modifier, (&pde_and_bc)); ///\todo amend as appropriate
        simulator.AddSimulationModifier(p_pde_modifier);

        // Run the simulation
        simulator.Solve();

        // Avoid memory leaks
        delete p_mesh;
    }
};

#endif /* TESTMUKULSIMULATION_HPP_ */
