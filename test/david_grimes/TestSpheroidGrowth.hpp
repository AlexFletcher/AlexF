#ifndef TESTSPHEROIDGROWTH_HPP_
#define TESTSPHEROIDGROWTH_HPP_

#include <cxxtest/TestSuite.h>

// Must be included before other cell_based headers
#include "CellBasedSimulationArchiver.hpp"

#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "CellLabel.hpp"
#include "SmartPointers.hpp"
#include "CellsGenerator.hpp"
#include "TransitCellProliferativeType.hpp"
#include "BernoulliTrialCellCycleModel.hpp"
#include "OnLatticeSimulation.hpp"
#include "CaBasedCellPopulation.hpp"
#include "ShovingCaBasedDivisionRule.hpp"
#include "PottsMeshGenerator.hpp"
#include "SpheroidDataWriter.hpp"
#include "PetscSetupAndFinalize.hpp"

static const double NUM_REPEATS = 100;

class TestSpheroidGrowth : public AbstractCellBasedWithTimingsTestSuite
{
public:

    void TestSpheroidGrowthWithContactInhibition() throw (Exception)
    {
        for (unsigned sim=0; sim<NUM_REPEATS; sim++)
        {
            // Initialise various singletons
            SimulationTime::Destroy();
            SimulationTime::Instance()->SetStartTime(0.0);
            CellPropertyRegistry::Instance()->Clear();
            CellId::ResetMaxCellId();
            RandomNumberGenerator::Instance()->Reseed(100.0*sim);

            // Create output directory
            std::stringstream out;
            out << sim;
            std::string output_directory = "WithContactInhibition/" +  out.str();

            // Create a mesh (i.e. lattice)
            double end_time = 50;
            unsigned domain_wide = 200;
            PottsMeshGenerator<3> generator(domain_wide, 0, 0, domain_wide, 0, 0, domain_wide, 0, 0);
            PottsMesh<3>* p_mesh = generator.GetMesh();

            // Find the centre of the mesh
            c_vector<double,3> centre_of_mesh = zero_vector<double>(3);
            for (unsigned i=0; i<p_mesh->GetNumNodes(); i++)
            {
                centre_of_mesh += p_mesh->GetNode(i)->rGetLocation();
            }
            centre_of_mesh /= p_mesh->GetNumNodes();

            // Specify the initial location of each cell
            std::vector<unsigned> location_indices;
            for (unsigned i=0; i<p_mesh->GetNumNodes(); i++)
            {
                if (norm_2(p_mesh->GetNode(i)->rGetLocation() - centre_of_mesh) < 1.0)
                {
                    location_indices.push_back(i);
                }
            }

            // Create cells
            std::vector<CellPtr> cells;
            MAKE_PTR(TransitCellProliferativeType, p_type);
            CellsGenerator<BernoulliTrialCellCycleModel,3> cells_generator;
            cells_generator.GenerateBasicRandom(cells, location_indices.size(), p_type);
            for (unsigned i=0; i<cells.size(); i++)
            {
                static_cast<BernoulliTrialCellCycleModel*>(cells[i]->GetCellCycleModel())->SetDivisionProbability(0.1);
                static_cast<BernoulliTrialCellCycleModel*>(cells[i]->GetCellCycleModel())->SetMinimumDivisionAge(0.1);
            }

            // Create cell population to maintain correspondence between cells and mesh
            CaBasedCellPopulation<3> cell_population(*p_mesh, cells, location_indices);
            cell_population.AddPopulationWriter<SpheroidDataWriter>();
            cell_population.SetOutputResultsForChasteVisualizer(false);

            // Create simulation to update cell states
            OnLatticeSimulation<3> simulator(cell_population);
            simulator.SetOutputDirectory(output_directory);
            simulator.SetDt(0.01);
            simulator.SetSamplingTimestepMultiple(100);
            simulator.SetEndTime(end_time);

            // Run simulation
            simulator.Solve();
        }
    }

    void TestSpheroidGrowthWithoutContactInhibition() throw (Exception)
    {
        for (unsigned sim=0; sim<NUM_REPEATS; sim++)
        {
            // Initialise various singletons
            SimulationTime::Destroy();
            SimulationTime::Instance()->SetStartTime(0.0);
            CellPropertyRegistry::Instance()->Clear();
            CellId::ResetMaxCellId();
            RandomNumberGenerator::Instance()->Reseed(100.0*sim);

            // Create output directory
            std::stringstream out;
            out << sim;
            std::string output_directory = "NoContactInhibition/" +  out.str();

            // Create a mesh (i.e. lattice)
            double end_time = 50;
            unsigned domain_wide = 200;
            PottsMeshGenerator<3> generator(domain_wide, 0, 0, domain_wide, 0, 0, domain_wide, 0, 0);
            PottsMesh<3>* p_mesh = generator.GetMesh();

            // Find the centre of the mesh
            c_vector<double,3> centre_of_mesh = zero_vector<double>(3);
            for (unsigned i=0; i<p_mesh->GetNumNodes(); i++)
            {
                centre_of_mesh += p_mesh->GetNode(i)->rGetLocation();
            }
            centre_of_mesh /= p_mesh->GetNumNodes();

            // Specify the initial location of each cell
            std::vector<unsigned> location_indices;
            for (unsigned i=0; i<p_mesh->GetNumNodes(); i++)
            {
                if (norm_2(p_mesh->GetNode(i)->rGetLocation() - centre_of_mesh) < 1.0)
                {
                    location_indices.push_back(i);
                }
            }

            // Create cells
            std::vector<CellPtr> cells;
            MAKE_PTR(TransitCellProliferativeType, p_type);
            CellsGenerator<BernoulliTrialCellCycleModel,3> cells_generator;
            cells_generator.GenerateBasicRandom(cells, location_indices.size(), p_type);
            for (unsigned i=0; i<cells.size(); i++)
            {
                static_cast<BernoulliTrialCellCycleModel*>(cells[i]->GetCellCycleModel())->SetDivisionProbability(0.1);
                static_cast<BernoulliTrialCellCycleModel*>(cells[i]->GetCellCycleModel())->SetMinimumDivisionAge(0.1);
            }

            // Create cell population to maintain correspondence between cells and mesh
            CaBasedCellPopulation<3> cell_population(*p_mesh, cells, location_indices);
            cell_population.AddPopulationWriter<SpheroidDataWriter>();
            cell_population.SetOutputResultsForChasteVisualizer(false);

            // Create simulation to update cell states
            OnLatticeSimulation<3> simulator(cell_population);
            simulator.SetOutputDirectory(output_directory);
            simulator.SetDt(0.01);
            simulator.SetSamplingTimestepMultiple(100);
            simulator.SetEndTime(end_time);

            // Add division rule
            boost::shared_ptr<AbstractCaBasedDivisionRule<3> > p_division_rule(new ShovingCaBasedDivisionRule<3>());
            cell_population.SetCaBasedDivisionRule(p_division_rule);

            // Run simulation
            simulator.Solve();
        }
    }
};

#endif /* TESTSPHEROIDGROWTH_HPP_ */
