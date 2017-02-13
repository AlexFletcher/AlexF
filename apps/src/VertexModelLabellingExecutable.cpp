
#include <iomanip>

// Includes from trunk
#include "ExecutableSupport.hpp"
#include "CellId.hpp"
#include "FakePetscSetup.hpp"
#include "SmartPointers.hpp"
#include "VoronoiVertexMeshGenerator.hpp"
#include "CellsGenerator.hpp"
#include "TransitCellProliferativeType.hpp"
#include "VertexBasedCellPopulation.hpp"
#include "OffLatticeSimulation.hpp"
#include "FarhadifarForceForAreaBasedCellCycleModel.hpp"
#include "CellAncestor.hpp"
#include "AreaBasedCellCycleModel.hpp"
#include "VolumeTrackingModifier.hpp"
#include "ClusterDataWriter.hpp"

// Program option includes for handling command line arguments
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/lexical_cast.hpp>

/*
 * Prototype functions
 */
void SetupSingletons(unsigned randomSeed);
void DestroySingletons();
void SetupAndRunSimulation(unsigned runIndex, unsigned numLabels, double labellingProportion);

int main(int argc, char *argv[])
{
    // This sets up PETSc and prints out copyright information, etc.
    ExecutableSupport::StandardStartup(&argc, &argv);

    // Define command line options
    boost::program_options::options_description general_options("This is the vertex model labelling executable.\n");
    general_options.add_options()
                    ("help", "produce help message")
                    ("L", boost::program_options::value<unsigned>()->default_value(1),"The number of coloured labels to use")
                    ("P", boost::program_options::value<double>()->default_value(0.1),"The proportion of cells in the tissue to label at time zero") ///\todo should this be float?
                    ("R", boost::program_options::value<unsigned>()->default_value(1),"The number of simulations to run for each parameter set")
                    ("S", boost::program_options::value<unsigned>()->default_value(0),"The random seed");

    // Define parse command line into variables_map
    boost::program_options::variables_map variables_map;
    boost::program_options::store(parse_command_line(argc, argv, general_options), variables_map);

    // Print help message if wanted
    if (variables_map.count("help"))
    {
        std::cout << std::setprecision(3) << general_options << "\n";
        std::cout << general_options << "\n";
        return 1;
    }

    // Get ID and name from command line
    unsigned num_labels = variables_map["L"].as<unsigned>();
    unsigned labelling_proportion = variables_map["P"].as<double>();
    unsigned num_runs = variables_map["R"].as<unsigned>();
    unsigned random_seed = variables_map["S"].as<unsigned>();

    for (unsigned run_index=0; run_index<num_runs; run_index++)
    {
        SetupSingletons(random_seed);
        SetupAndRunSimulation(run_index, num_labels, labelling_proportion);
        DestroySingletons();
    }
}

void SetupSingletons(unsigned randomSeed)
{
    // Set up what the test suite would do
    SimulationTime::Instance()->SetStartTime(0.0);

    // Reseed with 0 for same random numbers each time, or time(NULL) or simulation_id to change each realisation
    RandomNumberGenerator::Instance()->Reseed(randomSeed);
    CellPropertyRegistry::Instance()->Clear();
    CellId::ResetMaxCellId();
}

void DestroySingletons()
{
    // This is from the tearDown method of the test suite
    SimulationTime::Destroy();
    RandomNumberGenerator::Destroy();
    CellPropertyRegistry::Instance()->Clear();
}

void SetupAndRunSimulation(unsigned runIndex, unsigned numLabels, double labellingProportion)
{
    // Specify output directory (unique to each simulation)
    std::string output_directory = std::string("VertexModelLabelling")
        + std::string("_L") + boost::lexical_cast<std::string>(numLabels)
        + std::string("_P") + boost::lexical_cast<std::string>(labellingProportion)
        + std::string("_R") + boost::lexical_cast<std::string>(runIndex);

    // Set number of labels and waiting time
    double max_waiting_time = 48.0;

    /*
     * Set max cellular growth rate to 1/6 so that average growth rate is 1/12, hence
     * on average, a cell's target area will grow to twice its initial value
     * in 12 hours.
     */
    double max_growth_rate = 1.0/6.0;

    // Set parameters for initial tissue geometry
    unsigned num_cells_wide = 17;
    unsigned num_cells_high = 17;
    unsigned num_lloyd_steps = 3;
    double reference_target_area = 1.0;

    // Generate random initial mesh
    VoronoiVertexMeshGenerator generator(num_cells_wide, num_cells_high, num_lloyd_steps, reference_target_area);
    MutableVertexMesh<2,2>* p_mesh = generator.GetMesh();
    unsigned num_cells = p_mesh->GetNumElements();

    // Create a vector of cells and associate these with elements of the mesh
    std::vector<CellPtr> cells;
    MAKE_PTR(WildTypeCellMutationState, p_state);
    MAKE_PTR(TransitCellProliferativeType, p_type);
    for (unsigned elem_index=0; elem_index<num_cells; elem_index++)
    {
        AreaBasedCellCycleModel* p_model = new AreaBasedCellCycleModel();
        p_model->SetReferenceTargetArea(reference_target_area);
        p_model->SetMaxGrowthRate(max_growth_rate);

        CellPtr p_cell(new Cell(p_state, p_model));
        p_cell->SetCellProliferativeType(p_type);
        p_cell->SetBirthTime(-RandomNumberGenerator::Instance()->ranf()*12.0);
        p_cell->GetCellData()->SetItem("label", 0);

        cells.push_back(p_cell);
    }

    // Randomly label the correct number of cells, choosing a random colour in each case
    unsigned num_cells_to_label = labellingProportion*num_cells;
    unsigned labelling_index = 0;
    unsigned num_cells_labelled_so_far = 0;
    double u;
    while (num_cells_labelled_so_far < num_cells_to_label)
    {
        u = RandomNumberGenerator::Instance()->ranf();
        double r = (num_cells - labelling_index)*u;
        if (r < num_cells_to_label - num_cells_labelled_so_far)
        {
            // Choose a random colour and label the cell
            unsigned label = 1 + RandomNumberGenerator::Instance()->randMod(numLabels);
            cells[labelling_index]->GetCellData()->SetItem("label", label);
            num_cells_labelled_so_far++;
        }
        labelling_index++;
    }

    // Create cell population
    VertexBasedCellPopulation<2> cell_population(*p_mesh, cells);
    cell_population.SetOutputResultsForChasteVisualizer(false);
    cell_population.SetOutputCellRearrangementLocations(false);
    cell_population.SetCellAncestorsToLocationIndices();
    cell_population.AddPopulationWriter<ClusterDataWriter>();

    // Create simulation
    OffLatticeSimulation<2> simulation(cell_population);
    simulation.SetOutputDirectory(output_directory);
    simulation.SetSamplingTimestepMultiple(12.0/0.002); // Default time step is 0.002 for vertex models
    simulation.SetEndTime(max_waiting_time);

    // Pass in a force law
    MAKE_PTR(FarhadifarForceForAreaBasedCellCycleModel<2>, p_force);
    simulation.AddForce(p_force);

    // Pass in 'volume' (actually, area) modifier
    MAKE_PTR(VolumeTrackingModifier<2>, p_modifier);
    simulation.AddSimulationModifier(p_modifier);

    // Run simulation
    simulation.Solve();
}
