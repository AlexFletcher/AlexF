#ifndef TESTSIDEKICK_HPP_
#define TESTSIDEKICK_HPP_

#include <cxxtest/TestSuite.h>
#include "CheckpointArchiveTypes.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "HoneycombVertexMeshGenerator.hpp"
#include "CellsGenerator.hpp"
#include "NoCellCycleModel.hpp"
#include "VertexBasedCellPopulation.hpp"
#include "StripeStatisticsWriter.hpp"
#include "ForceForScenario1.hpp"
#include "ConstantTargetAreaModifier.hpp"
#include "OffLatticeSimulation.hpp"
#include "SmartPointers.hpp"
#include "FakePetscSetup.hpp"
#include "RandomForce.hpp"
#include "CellStripesWriter.hpp"

class TestSidekick : public AbstractCellBasedWithTimingsTestSuite
{
public:

    void TestSimulationWithoutSupercontractility() throw (Exception)
    {
        // Specify simulation rules
        bool include_random_jiggling = false;
        bool check_internal_intersections = false;
        bool use_combined_interfaces_for_line_tension = false;
        bool use_distinct_stripe_mismatches_for_combined_interfaces = false;

        // Specify mechanical parameter values
        double k = 1.0;
        double lambda_bar = 0.05;
        double gamma_bar = 0.04;
        double heterotypic_line_tension_multiplier = 2.0;
        double supercontractile_line_tension_multiplier = 2.0;

        // Diffusion constant is only used if include_random_jiggling == true
        double diffusion_constant = 0.01;

        // Specify pre-stripe mechanical relaxation time and stripe simulation time
        double relaxation_time = 100;//200.0;
        double stripe_simulation_time = 250;//500.0;

        // Specify tissue geometry
        unsigned num_cells_wide = 14;
        unsigned num_cells_high = 20;

        // Specify time step
        double time_step = 0.01;
        double output_time_step = 1.0;

        // Initialise various singletons
        SimulationTime::Destroy();
        SimulationTime::Instance()->SetStartTime(0.0);
        CellPropertyRegistry::Instance()->Clear();
        CellId::ResetMaxCellId();

        // Generate a vertex mesh
        HoneycombVertexMeshGenerator honeycomb_generator(num_cells_wide, num_cells_high);
        MutableVertexMesh<2,2>* p_mesh = honeycomb_generator.GetMesh();
        p_mesh->SetCheckForInternalIntersections(check_internal_intersections);

        // Create some non-proliferating cells
        std::vector<CellPtr> cells;
        CellsGenerator<NoCellCycleModel, 2> cells_generator;
        cells_generator.GenerateBasic(cells, p_mesh->GetNumElements());

        // Bestow cell stripe identities
        for (unsigned i=0; i<cells.size(); i++)
        {
            cells[i]->GetCellData()->SetItem("stripe", 1);
        }

        // Create a cell population that associates the cells with the vertex mesh
        VertexBasedCellPopulation<2> cell_population(*p_mesh, cells);
        cell_population.SetOutputResultsForChasteVisualizer(true);
        cell_population.SetOutputCellRearrangementLocations(false);
        cell_population.AddCellWriter<CellStripesWriter>();

        // Create a simulation using the cell population
        OffLatticeSimulation<2> simulation(cell_population);
        simulation.SetOutputDirectory("Scenario1");
        simulation.SetEndTime(relaxation_time);

        simulation.SetDt(time_step);
        unsigned output_time_step_multiple = (unsigned) (output_time_step/time_step);
        simulation.SetSamplingTimestepMultiple(output_time_step_multiple);

        // Create the appropriate force law(s) for the specified geometry
        MAKE_PTR(ForceForScenario1<2>, p_force);
        p_force->SetNumStripes(4);
        p_force->SetAreaElasticityParameter(k);
        p_force->SetPerimeterContractilityParameter(gamma_bar*k);
        p_force->SetHomotypicLineTensionParameter(lambda_bar*pow(k,1.5));
        p_force->SetHeterotypicLineTensionParameter(heterotypic_line_tension_multiplier*lambda_bar*pow(k,1.5));
        p_force->SetSupercontractileLineTensionParameter(supercontractile_line_tension_multiplier*lambda_bar*pow(k,1.5));
        p_force->SetBoundaryLineTensionParameter(lambda_bar*lambda_bar*pow(k,1.5));
        p_force->SetUseCombinedInterfacesForLineTension(false);

        simulation.AddForce(p_force);

        if (include_random_jiggling)
        {
            MAKE_PTR(RandomForce<2>, p_random_force);
            p_random_force->SetDiffusionConstant(diffusion_constant);
            simulation.AddForce(p_random_force);
        }

        // Pass in a target area modifier (needed, but not used)
        MAKE_PTR(ConstantTargetAreaModifier<2>, p_growth_modifier);
        simulation.AddSimulationModifier(p_growth_modifier);

        // Run simulation
        simulation.Solve();

        // Bestow cell stripe identities
        for (unsigned i=0; i<simulation.rGetCellPopulation().GetNumRealCells(); i++)
        {
            unsigned row = i/num_cells_wide;
            unsigned col = i%num_cells_wide;

            CellPtr p_cell = simulation.rGetCellPopulation().GetCellUsingLocationIndex(i);
            if (row%4 == 0)
            {
                if ((col%7 == 0) || (col%7 == 4))      { p_cell->GetCellData()->SetItem("stripe", 1); }
                else if ((col%7 == 1) || (col%7 == 5)) { p_cell->GetCellData()->SetItem("stripe", 2); }
                else if ((col%7 == 2) || (col%7 == 6)) { p_cell->GetCellData()->SetItem("stripe", 3); }
                else                                   { p_cell->GetCellData()->SetItem("stripe", 4); }
            }
            else if (row%4 == 1)
            {
                if ((col%7 == 0) || (col%7 == 3))      { p_cell->GetCellData()->SetItem("stripe", 1); }
                else if ((col%7 == 1) || (col%7 == 4)) { p_cell->GetCellData()->SetItem("stripe", 2); }
                else if (col%7 == 5)                   { p_cell->GetCellData()->SetItem("stripe", 3); }
                else                                   { p_cell->GetCellData()->SetItem("stripe", 4); }
            }
            else if (row%4 == 2)
            {
                if ((col%7 == 0) || (col%7 == 4))      { p_cell->GetCellData()->SetItem("stripe", 1); }
                else if ((col%7 == 1) || (col%7 == 5)) { p_cell->GetCellData()->SetItem("stripe", 2); }
                else if (col%7 == 2)                   { p_cell->GetCellData()->SetItem("stripe", 3); }
                else                                   { p_cell->GetCellData()->SetItem("stripe", 4); }
            }
            else
            {
                if ((col%7 == 0) || (col%7 == 3))      { p_cell->GetCellData()->SetItem("stripe", 1); }
                else if ((col%7 == 1) || (col%7 == 4)) { p_cell->GetCellData()->SetItem("stripe", 2); }
                else if ((col%7 == 2) || (col%7 == 5)) { p_cell->GetCellData()->SetItem("stripe", 3); }
                else                                   { p_cell->GetCellData()->SetItem("stripe", 4); }
            }
        }

        p_force->SetUseCombinedInterfacesForLineTension(use_combined_interfaces_for_line_tension);
        p_force->SetUseDistinctStripeMismatchesForCombinedInterfaces(use_distinct_stripe_mismatches_for_combined_interfaces);

        simulation.SetEndTime(stripe_simulation_time + relaxation_time);
        simulation.Solve();
    }
};

#endif /* TESTSIDEKICK_HPP_*/

