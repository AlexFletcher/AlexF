
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
#include "LongAxisVertexBasedDivisionRule.hpp"
#include "OffLongAxisVertexBasedDivisionRule.hpp"
#include "TensionOrientedVertexBasedDivisionRule.hpp"

#include "OffLatticeSimulation.hpp"
#include "FarhadifarForce.hpp"
#include "ConstantTargetAreaModifier.hpp"
#include "TargetAreaLinearGrowthModifier.hpp"
#include "CellPackingDataWriter.hpp"
#include "FollicularEpitheliumStretchModifier.hpp"

class TestFollicularEpitheliumCellPacking : public AbstractCellBasedWithTimingsTestSuite
{
private:

    void RunSimulations(unsigned divisionRule, unsigned stretch, unsigned numSimulations)
    {
        RandomNumberGenerator* p_gen = RandomNumberGenerator::Instance();

        // Set parameters
        double mean_g1_phase = 2.0;
        double s_phase = 1.0;
        double g2_phase = 1.5;
        double m_phase = 0.5;
        double mean_cycle = mean_g1_phase + s_phase + g2_phase + m_phase;

        double time_step = 0.0005;
        double simulation_duration = 30.0;

        // Set parameters for initial tissue geometry
        unsigned num_cells_wide = 5;
        unsigned num_cells_high = 5;
        unsigned num_lloyd_steps = 3;

        for (unsigned sim_index=0; sim_index<numSimulations; sim_index++)
        {
            // Generate the name of the output directory based on the input arguments to this method
            std::stringstream out;
            out << "TestFollicularEpitheliumCellPacking/";
            switch (divisionRule)
            {
                case 0:  { out << "RandomOrientedDivision";      break; }
                case 1:  { out << "ShortAxisOrientedDivision";   break; }
                case 2:  { out << "LongAxisOrientedDivision";    break; }
                case 3:  { out << "OffLongAxisOrientedDivision"; break; }
                case 4:  { out << "TensionOrientedDivision";     break; }
                default: { NEVER_REACHED; }
            }
            switch (stretch)
            {
                case 0:  { out << "NoStretch";         break; }
                case 1:  { out << "UniformStretch";    break; }
                case 2:  { out << "NonUniformStretch"; break; }
                default: { NEVER_REACHED; }
            }
            out << "/Sim" << sim_index;
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

            // Set the rule for cell division orientation
            switch (divisionRule)
            {
                case 0:
                {
                    MAKE_PTR(RandomDirectionVertexBasedDivisionRule<2>, p_random_rule);
                    cell_population.SetVertexBasedDivisionRule(p_random_rule);
                    break;
                }
                case 1:
                {
                    MAKE_PTR(ShortAxisVertexBasedDivisionRule<2>, p_short_axis_rule);
                    cell_population.SetVertexBasedDivisionRule(p_short_axis_rule);
                    break;
                }
                case 2:
                {
                    MAKE_PTR(LongAxisVertexBasedDivisionRule<2>, p_long_axis_rule);
                    cell_population.SetVertexBasedDivisionRule(p_long_axis_rule);
                    break;
                }
                case 3:
                {
                    MAKE_PTR(OffLongAxisVertexBasedDivisionRule<2>, p_off_long_axis_rule);
                    cell_population.SetVertexBasedDivisionRule(p_off_long_axis_rule);
                    break;
                }
                case 4:
                {
                    MAKE_PTR(TensionOrientedVertexBasedDivisionRule<2>, p_tension_rule);
                    cell_population.SetVertexBasedDivisionRule(p_tension_rule);
                    break;
                }
                default: { NEVER_REACHED; }
            }

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

            ///\todo impose stretch, if specified

            // Run simulation
            simulation.Solve();
        }
    }

public:

    void TestRandomOrientedDivisionNoStretch10Simulations() throw (Exception)
    {
        RunSimulations(0, 0, 5);
    }

    void TestShortAxisOrientedDivisionNoStretch10Simulations() throw (Exception)
    {
        RunSimulations(1, 0, 5);
    }

    void TestLongAxisOrientedDivisionNoStretch10Simulations() throw (Exception)
    {
        RunSimulations(2, 0, 5);
    }

    void TestOffLongAxisOrientedDivisionNoStretch10Simulations() throw (Exception)
    {
        RunSimulations(3, 0, 5);
    }
};

#endif /* TESTFOLLICULAREPITHELIUMCELLPACKING_HPP_ */
