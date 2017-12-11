
#ifndef TESTFOLLICULAREPITHELIUMCELLPACKING_HPP_
#define TESTFOLLICULAREPITHELIUMCELLPACKING_HPP_

#include <cxxtest/TestSuite.h>
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "FakePetscSetup.hpp"
#include "SmartPointers.hpp"

#include "VoronoiVertexMeshGenerator.hpp"
#include "WildTypeCellMutationState.hpp"
#include "TransitCellProliferativeType.hpp"
#include "UniformCellCycleModel.hpp"
#include "VertexBasedCellPopulation.hpp"

#include "RandomDirectionVertexBasedDivisionRule.hpp"
#include "ShortAxisVertexBasedDivisionRule.hpp"
#include "LongAxisVertexBasedDivisionRule.hpp"
#include "OffLongAxisVertexBasedDivisionRule.hpp"
#include "TensionOrientedVertexBasedDivisionRule.hpp"
#include "OffTissueAxisVertexBasedDivisionRule.hpp"

#include "OffLatticeSimulation.hpp"
#include "FarhadifarForce.hpp"
#include "ConstantTargetAreaModifier.hpp"
#include "TargetAreaLinearGrowthModifier.hpp"
#include "CellPackingDataWriter.hpp"
#include "FollicularEpitheliumStretchModifier.hpp"
#include "ExtrinsicPullModifier.hpp"
#include "AreaBasedCellCycleModel.hpp"
#include "TargetAreaModifierForAreaBasedCellCycleModel.hpp"
#include "ExponentialG1GenerationalCellCycleModel.hpp"
#include "VolumeTrackingModifier.hpp"
#include "CellAgesWriter.hpp"
#include "DifferentiatedCellProliferativeType.hpp"
#include "Toroidal2dVertexMeshStretchModifier.hpp"

class TestFollicularEpitheliumCellPacking : public AbstractCellBasedWithTimingsTestSuite
{
public:

    void XTestSuccessfulSimulation() throw (Exception)
    {
        RandomNumberGenerator* p_gen = RandomNumberGenerator::Instance();

        double time_step = 0.002;
        double simulation_duration = 500.0;

        // Set parameters for initial tissue geometry
        unsigned num_cells_wide = 5;
        unsigned num_cells_high = 5;
        unsigned num_lloyd_steps = 1;

        // Generate the name of the output directory based on the input arguments to this method
        std::stringstream out;
        out << "EveningSimulation";
        std::string output_directory = out.str();
        OutputFileHandler results_handler(output_directory, false);

        // Initialise various singletons
        SimulationTime::Destroy();
        SimulationTime::Instance()->SetStartTime(0.0);
        CellPropertyRegistry::Instance()->Clear();
        CellId::ResetMaxCellId();
        p_gen->Reseed(0);

        // Generate random initial mesh
        VoronoiVertexMeshGenerator generator(num_cells_wide, num_cells_high, num_lloyd_steps);
//        MutableVertexMesh<2,2>* p_mesh = generator.GetMesh();
        Toroidal2dVertexMesh* p_mesh = generator.GetToroidalMesh();
        unsigned num_cells = p_mesh->GetNumElements();

        // Create a vector of cells and associate these with elements of the mesh
        std::vector<CellPtr> cells;
        MAKE_PTR(WildTypeCellMutationState, p_state);
//        MAKE_PTR(TransitCellProliferativeType, p_type);
        MAKE_PTR(DifferentiatedCellProliferativeType, p_type);
        for (unsigned elem_index=0; elem_index<num_cells; elem_index++)
        {
//            ExponentialG1GenerationalCellCycleModel* p_model = new ExponentialG1GenerationalCellCycleModel();
//            double mean_g1_phase = 20.0;
//            p_model->SetRate(1.0/mean_g1_phase); ///\todo set suitable mean G1 duration and other phases' durations
//            p_model->SetSDuration(1.0);
//            p_model->SetG2Duration(2.0);
//            p_model->SetMDuration(1.0);
//            p_model->SetMaxTransitGenerations(UINT_MAX);

//                AreaBasedCellCycleModel* p_model = new AreaBasedCellCycleModel();
//                p_model->SetDimension(2);
//                p_model->SetReferenceTargetArea(1.0);
//                p_model->SetMaxGrowthRate(1.0/60.0);

                UniformCellCycleModel* p_model = new UniformCellCycleModel();
                p_model->SetDimension(2);
                p_model->SetMinCellCycleDuration(72.0);
                p_model->SetMaxCellCycleDuration(120.0);

            CellPtr p_cell(new Cell(p_state, p_model));
            p_cell->SetCellProliferativeType(p_type);
            p_cell->SetBirthTime(-48.0*RandomNumberGenerator::Instance()->ranf());
            cells.push_back(p_cell);
        }

        // Create cell population
        VertexBasedCellPopulation<2> cell_population(*p_mesh, cells);
        cell_population.SetOutputResultsForChasteVisualizer(false);
        cell_population.SetOutputCellRearrangementLocations(false);
        cell_population.AddCellWriter<CellPackingDataWriter>();
        cell_population.AddCellWriter<CellAgesWriter>();

        // Set the rule for cell division orientation
        unsigned divisionRule = 0;
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
            case 5:
            {
                MAKE_PTR(OffTissueAxisVertexBasedDivisionRule<2>, p_tissue_rule);
                cell_population.SetVertexBasedDivisionRule(p_tissue_rule);
                break;
            }
            default:
            {
                NEVER_REACHED;
            }
        }

        // Create simulation
        OffLatticeSimulation<2> simulation(cell_population);
        simulation.SetOutputDirectory(output_directory);
        simulation.SetDt(time_step);
        simulation.SetSamplingTimestepMultiple(10.0/time_step);
        simulation.SetEndTime(50.0);

        // Pass in a force law
        MAKE_PTR(FarhadifarForce<2>, p_force);
//        p_force->SetPerimeterContractilityParameter(0.04/2);//0.04
//        p_force->SetLineTensionParameter(2*0.12);//0.12
        simulation.AddForce(p_force);

        // Pass in a target area modifier
        MAKE_PTR(ConstantTargetAreaModifier<2>, p_modifier);
//        MAKE_PTR(TargetAreaLinearGrowthModifier<2>, p_modifier);
//        MAKE_PTR(TargetAreaModifierForAreaBasedCellCycleModel<2>, p_modifier);
        simulation.AddSimulationModifier(p_modifier);

//            MAKE_PTR(VolumeTrackingModifier<2>, p_vol_modifier);
//            simulation.AddSimulationModifier(p_vol_modifier);

        // Run simulation
        simulation.Solve();

        for (AbstractCellPopulation<2,2>::Iterator cell_iter = simulation.rGetCellPopulation().Begin();
             cell_iter != simulation.rGetCellPopulation().End();
             ++cell_iter)
        {
            boost::shared_ptr<AbstractCellProperty> p_type = simulation.rGetCellPopulation().GetCellPropertyRegistry()->template Get<TransitCellProliferativeType>();
            cell_iter->SetCellProliferativeType(p_type);
            dynamic_cast<AbstractSimpleCellCycleModel*>(cell_iter->GetCellCycleModel())->SetCellCycleDuration();
        }
        simulation.SetEndTime(50.0+simulation_duration);

        // Set the tissue stretch rule
        MAKE_PTR(Toroidal2dVertexMeshStretchModifier, p_stretch_modifier);
//        MAKE_PTR(FollicularEpitheliumStretchModifier<2>, p_stretch_modifier);
//        p_stretch_modifier->ApplyExtrinsicPullToAllNodes(true);
        p_stretch_modifier->SetSpeed(0.005);
        simulation.AddSimulationModifier(p_stretch_modifier);

        simulation.Solve();
    }

    void TestSimulation() throw (Exception)
    {
        RandomNumberGenerator* p_gen = RandomNumberGenerator::Instance();

        double time_step = 0.0002;

        // Set parameters for initial tissue geometry
        unsigned num_cells_wide = 6;
        unsigned num_cells_high = 6;
        unsigned num_lloyd_steps = 0;

        // Initialise various singletons
        SimulationTime::Destroy();
        SimulationTime::Instance()->SetStartTime(0.0);
        CellPropertyRegistry::Instance()->Clear();
        CellId::ResetMaxCellId();
        p_gen->Reseed(0);

        // Generate random initial mesh
        VoronoiVertexMeshGenerator generator(num_cells_wide, num_cells_high, num_lloyd_steps);
        Toroidal2dVertexMesh* p_mesh = generator.GetToroidalMesh();
        unsigned num_cells = p_mesh->GetNumElements();

        // Create a vector of cells and associate these with elements of the mesh
        std::vector<CellPtr> cells;
        MAKE_PTR(WildTypeCellMutationState, p_state);
        MAKE_PTR(DifferentiatedCellProliferativeType, p_type);
        for (unsigned elem_index=0; elem_index<num_cells; elem_index++)
        {
            UniformCellCycleModel* p_model = new UniformCellCycleModel();
            p_model->SetDimension(2);
            p_model->SetMinCellCycleDuration(3.0);
            p_model->SetMaxCellCycleDuration(5.0);

            CellPtr p_cell(new Cell(p_state, p_model));
            p_cell->SetCellProliferativeType(p_type);
            p_cell->SetBirthTime(-48.0*RandomNumberGenerator::Instance()->ranf());
            cells.push_back(p_cell);
        }

        // Create cell population
        VertexBasedCellPopulation<2> cell_population(*p_mesh, cells);
        cell_population.SetOutputResultsForChasteVisualizer(false);
        cell_population.SetOutputCellRearrangementLocations(false);
        cell_population.AddCellWriter<CellPackingDataWriter>();
        cell_population.AddCellWriter<CellAgesWriter>();
        cell_population.SetDampingConstantNormal(0.04);

        // Set the rule for cell division orientation
//        MAKE_PTR(RandomDirectionVertexBasedDivisionRule<2>, p_rule);
//        MAKE_PTR(ShortAxisVertexBasedDivisionRule<2>, p_rule);
//        MAKE_PTR(LongAxisVertexBasedDivisionRule<2>, p_rule);
//        MAKE_PTR(OffLongAxisVertexBasedDivisionRule<2>, p_rule);
//        MAKE_PTR(TensionOrientedVertexBasedDivisionRule<2>, p_rule);
        MAKE_PTR(OffTissueAxisVertexBasedDivisionRule<2>, p_rule);
        cell_population.SetVertexBasedDivisionRule(p_rule);

        // Create simulation
        OffLatticeSimulation<2> simulation(cell_population);
        simulation.SetOutputDirectory("OffTissueAxis");
        simulation.SetDt(time_step);
        simulation.SetSamplingTimestepMultiple(1.0/time_step);
        simulation.SetEndTime(2.0);

        // Pass in a force law
        MAKE_PTR(FarhadifarForce<2>, p_force);
        p_force->SetPerimeterContractilityParameter(0.04);
        p_force->SetLineTensionParameter(0.12);
        simulation.AddForce(p_force);

        // Pass in a target area modifier
        MAKE_PTR(ConstantTargetAreaModifier<2>, p_modifier);
        simulation.AddSimulationModifier(p_modifier);

        // Run simulation
        simulation.Solve();

        for (AbstractCellPopulation<2,2>::Iterator cell_iter = simulation.rGetCellPopulation().Begin();
             cell_iter != simulation.rGetCellPopulation().End();
             ++cell_iter)
        {
            boost::shared_ptr<AbstractCellProperty> p_type = simulation.rGetCellPopulation().GetCellPropertyRegistry()->template Get<TransitCellProliferativeType>();
            cell_iter->SetCellProliferativeType(p_type);
            dynamic_cast<AbstractSimpleCellCycleModel*>(cell_iter->GetCellCycleModel())->SetCellCycleDuration();
        }
        simulation.SetEndTime(32.0);

        // Set the tissue stretch rule
        MAKE_PTR(Toroidal2dVertexMeshStretchModifier, p_stretch_modifier);
        p_stretch_modifier->SetSpeed(0.02);
        simulation.AddSimulationModifier(p_stretch_modifier);

        simulation.Solve();
    }
};

#endif /* TESTFOLLICULAREPITHELIUMCELLPACKING_HPP_ */
