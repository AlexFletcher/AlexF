
#ifndef TESTFOLLICULAREPITHELIUMCELLPACKING_HPP_
#define TESTFOLLICULAREPITHELIUMCELLPACKING_HPP_

#include <cxxtest/TestSuite.h>
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "FakePetscSetup.hpp"
#include "SmartPointers.hpp"
#include "VoronoiVertexMeshGenerator.hpp"
#include "WildTypeCellMutationState.hpp"
#include "TransitCellProliferativeType.hpp"
#include "ExponentialG1GenerationalCellCycleModel.hpp"
#include "VertexBasedCellPopulation.hpp"
#include "RandomDirectionVertexBasedDivisionRule.hpp"
#include "ShortAxisVertexBasedDivisionRule.hpp"
#include "OffLatticeSimulation.hpp"
#include "FarhadifarForce.hpp"
#include "ConstantTargetAreaModifier.hpp"
#include "TargetAreaLinearGrowthModifier.hpp"
#include "CellPackingDataWriter.hpp"
#include "FollicularEpitheliumStretchModifier.hpp"

class TestFollicularEpitheliumCellPacking : public AbstractCellBasedWithTimingsTestSuite
{
public:

    void TestSimulation() throw(Exception)
    {
        RandomNumberGenerator* p_gen = RandomNumberGenerator::Instance();

        // Set parameters
        double mean_g1_phase = 2.0;
        double s_phase = 1.0;
        double g2_phase = 0.5;
        double m_phase = 0.5;
        double mean_cycle = mean_g1_phase + s_phase + g2_phase + m_phase;

        double time_step = 0.001;
        unsigned num_simulations = 1;
        double simulation_duration = 40.0; // 30.0

        // Set parameters for initial tissue geometry
        unsigned num_cells_wide = 5;
        unsigned num_cells_high = 5;
        unsigned num_lloyd_steps = 3;

        for (unsigned sim_index=0; sim_index<num_simulations; sim_index++)
        {
            // Specify output directory
            std::stringstream out;
            out << "TestFollicularEpitheliumCellPacking" << "/Sim" << sim_index;
            std::string output_directory = out.str();
            OutputFileHandler results_handler(output_directory, false);

            // Initialise various singletons
            SimulationTime::Destroy();
            SimulationTime::Instance()->SetStartTime(0.0);
            CellPropertyRegistry::Instance()->Clear();
            CellId::ResetMaxCellId();
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
                ExponentialG1GenerationalCellCycleModel* p_model = new ExponentialG1GenerationalCellCycleModel();
                p_model->SetDimension(2);
                p_model->SetRate(1.0/mean_g1_phase); ///\todo set suitable mean G1 duration and other phases' durations
                p_model->SetSDuration(s_phase);
                p_model->SetG2Duration(g2_phase);
                p_model->SetMDuration(m_phase);
                p_model->SetMaxTransitGenerations(UINT_MAX);

                CellPtr p_cell(new Cell(p_state, p_model));
                p_cell->SetCellProliferativeType(p_type);
                p_cell->SetBirthTime(0.0);
                cells.push_back(p_cell);
            }

            // Create cell population
            VertexBasedCellPopulation<2> cell_population(*p_mesh, cells);
            cell_population.SetDampingConstantNormal(0.1);
            cell_population.SetOutputResultsForChasteVisualizer(false);
            cell_population.SetOutputCellRearrangementLocations(false);
            cell_population.AddCellWriter<CellPackingDataWriter>();

            // Set the division rule for our population
            boost::shared_ptr<AbstractVertexBasedDivisionRule<2> > p_division_rule_to_set(new RandomDirectionVertexBasedDivisionRule<2>());
            cell_population.SetVertexBasedDivisionRule(p_division_rule_to_set);

            // Create simulation
            OffLatticeSimulation<2> simulation(cell_population);
            simulation.SetOutputDirectory(output_directory);
            simulation.SetDt(time_step);
            simulation.SetSamplingTimestepMultiple(mean_cycle/time_step);
            simulation.SetEndTime(simulation_duration);

            // Pass in a force law
            MAKE_PTR(FarhadifarForce<2>, p_force);
            simulation.AddForce(p_force);

            // Pass in a target area modifier
            MAKE_PTR(ConstantTargetAreaModifier<2>, p_modifier);
            simulation.AddSimulationModifier(p_modifier);

            // Run simulation
            simulation.Solve();

            std::cout << simulation.rGetCellPopulation().GetNumRealCells() << std::endl;
        }
    }
};

#endif /* TESTFOLLICULAREPITHELIUMCELLPACKING_HPP_ */
