#ifndef MUKULPDESYSTEMSOLVER_HPP_
#define MUKULPDESYSTEMSOLVER_HPP_

#include "ChasteSerialization.hpp"
#include <boost/serialization/base_object.hpp>

#include "AbstractCellBasedSimulationModifier.hpp"
#include "AbstractAssemblerSolverHybrid.hpp"
#include "AbstractDynamicLinearPdeSolver.hpp"
#include "SimpleLinearParabolicSolver.hpp"
#include "BoundaryConditionsContainer.hpp"
#include "MukulPdeSystem.hpp"
#include "VtkMeshWriter.hpp"

template<unsigned DIM>
class MukulPdeSystemSolver : public AbstractCellBasedSimulationModifier<DIM>,
                             public AbstractAssemblerSolverHybrid<DIM, DIM, 2, NORMAL>,
                             public AbstractDynamicLinearPdeSolver<DIM, DIM, 2>
{
protected:

    /**
     * Shared pointer to a linear PDE object.
     */
    boost::shared_ptr<MukulPdeSystem<DIM> > mpPdeSystem;

    BoundaryConditionsContainer<DIM, DIM, 2>* mpBoundaryConditions;

    /** The solution to the PDE problem at the current time step. */
    Vec mSolution;

    /** Pointer to the finite element mesh on which to solve the PDE. */
    TetrahedralMesh<DIM,DIM>* mpMesh;

    /** Store the output directory name. */
    std::string mOutputDirectory;

    /** File that the values of the PDE solution are written out to. */
    out_stream mpVizPdeSolutionResultsFile;

    /**
     * Whether to delete the finite element mesh when we are destroyed.
     */
    bool mDeleteFeMesh;

    std::map<CellPtr, unsigned> mCellPdeElementMap;

    void SetupLinearSystem(Vec currentSolution, bool computeMatrix);

public:

    MukulPdeSystemSolver(boost::shared_ptr<MukulPdeSystem<DIM>> pPdeSystem,
    		             BoundaryConditionsContainer<DIM,DIM,2>* pBoundaryConditions,
    		             TetrahedralMesh<DIM,DIM>* pMesh,
						 Vec solution=nullptr);

    /**
     * Destructor.
     */
    virtual ~MukulPdeSystemSolver();

    /**
     * Overridden UpdateAtEndOfTimeStep() method.
     *
     * Specifies what to do in the simulation at the end of each time step.
     *
     * @param rCellPopulation reference to the cell population
     */
    virtual void UpdateAtEndOfTimeStep(AbstractCellPopulation<DIM,DIM>& rCellPopulation);

    virtual void UpdateAtEndOfOutputTimeStep(AbstractCellPopulation<DIM,DIM>& rCellPopulation);

    virtual void UpdateAtEndOfSolve(AbstractCellPopulation<DIM,DIM>& rCellPopulation);

    virtual void SetupSolve(AbstractCellPopulation<DIM,DIM>& rCellPopulation, std::string outputDirectory);

    /**
     * Helper method to initialise the PDE solution using the CellData.
     *
     * Here we assume a homogeneous initial consition.
     *
     * @param rCellPopulation reference to the cell population
     */
    void SetupInitialSolutionVector(AbstractCellPopulation<DIM,DIM>& rCellPopulation);

    /**
     * Overridden OutputSimulationModifierParameters() method.
     * Output any simulation modifier parameters to file.
     *
     * @param rParamsFile the file stream to which the parameters are output
     */
    void OutputSimulationModifierParameters(out_stream& rParamsFile);

    /**
     * Helper method to copy the PDE solution to CellData
     *
     * Here we need to interpolate from the FE mesh onto the cells.
     *
     * @param rCellPopulation reference to the cell population
     */
    void UpdateCellData(AbstractCellPopulation<DIM,DIM>& rCellPopulation);

    /**
     * Initialise mCellPdeElementMap.
     *
     * @param rCellPopulation reference to the cell population
     */
    void InitialiseCellPdeElementMap(AbstractCellPopulation<DIM,DIM>& rCellPopulation);

    /**
     * Update the mCellPdeElementMap
     *
     * This method should be called before sending the element map to a PDE class
     * to ensure map is up to date.
     *
     * @param rCellPopulation reference to the cell population
     */
    void UpdateCellPdeElementMap(AbstractCellPopulation<DIM,DIM>& rCellPopulation);
};

template<unsigned DIM>
MukulPdeSystemSolver<DIM>::MukulPdeSystemSolver(boost::shared_ptr<MukulPdeSystem<DIM>> pPdeSystem,
		                                        BoundaryConditionsContainer<DIM, DIM, 2>* pBoundaryConditions,
		                                        TetrahedralMesh<DIM,DIM>* pMesh,
											    Vec solution)
    : AbstractCellBasedSimulationModifier<DIM>(),
      AbstractAssemblerSolverHybrid<DIM, DIM, 2, NORMAL>(pMesh, pBoundaryConditions),
      AbstractDynamicLinearPdeSolver<DIM, DIM, 2>(pMesh),
      mpPdeSystem(pPdeSystem),
	  mSolution(nullptr),
      mpMesh(pMesh),
	  mOutputDirectory("")
{
    this->mpBoundaryConditions = pBoundaryConditions;

    if (solution)
    {
        mSolution = solution;
    }
}

template<unsigned DIM>
MukulPdeSystemSolver<DIM>::~MukulPdeSystemSolver()
{
    if (mDeleteFeMesh and mpMesh!=nullptr)
    {
        delete mpMesh;
    }
    if (mSolution)
    {
        PetscTools::Destroy(mSolution);
    }
}

template<unsigned DIM>
void MukulPdeSystemSolver<DIM>::UpdateAtEndOfTimeStep(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
{
    this->UpdateCellPdeElementMap(rCellPopulation);

    ///\todo Investigate more than one PDE time step per spatial step
    SimulationTime* p_simulation_time = SimulationTime::Instance();
    double current_time = p_simulation_time->GetTime();
    double dt = p_simulation_time->GetTimeStep();
    this->SetTimes(current_time,current_time + dt);
    this->SetTimeStep(dt);

    // Use previous solution as the initial condition
    Vec previous_solution = this->mSolution;
    this->mInitialCondition = previous_solution;

    // Note that the linear solver creates a vector, so we have to keep a handle on the old one
    // in order to destroy it
    this->mSolution = this->Solve();
    PetscTools::Destroy(previous_solution);
    this->UpdateCellData(rCellPopulation);

    // Write solution to VTK
//    WriteVtkResultsToFile(soln, stepper.GetTotalTimeStepsTaken());
}

template<unsigned DIM>
void MukulPdeSystemSolver<DIM>::SetupSolve(AbstractCellPopulation<DIM,DIM>& rCellPopulation, std::string outputDirectory)
{
    // Cache the output directory
    this->mOutputDirectory = outputDirectory;

    InitialiseCellPdeElementMap(rCellPopulation);

    // Copy the cell data to mSolution (this is the initial condition)
    SetupInitialSolutionVector(rCellPopulation);

    // Output the initial conditions on FeMesh
    this->UpdateAtEndOfOutputTimeStep(rCellPopulation);
}

template<unsigned DIM>
void MukulPdeSystemSolver<DIM>::SetupInitialSolutionVector(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
{
    // Specify homogeneous initial conditions based upon the values stored in CellData.
    // Note need all the CellDataValues to be the same.

	double initial_condition = rCellPopulation.Begin()->GetCellData()->GetItem("bmp");

	for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = rCellPopulation.Begin();
		 cell_iter != rCellPopulation.End();
		 ++cell_iter)
	{
		double initial_condition_at_cell = cell_iter->GetCellData()->GetItem("bmp");
		UNUSED_OPT(initial_condition_at_cell);
		assert(fabs(initial_condition_at_cell - initial_condition) < 1e-12);
	}
///\todo how does LinPaPdOdeSysSolver deal with the structure of Vec soln?
    // Initialise mSolution
    this->mSolution = PetscTools::CreateAndSetVec(this->mpMesh->GetNumNodes(), initial_condition);
}

template<unsigned DIM>
void MukulPdeSystemSolver<DIM>::OutputSimulationModifierParameters(out_stream& rParamsFile)
{
    // No parameters to output, so just call method on direct parent class
    MukulPdeSystemSolver<DIM>::OutputSimulationModifierParameters(rParamsFile);
}

template<unsigned DIM>
void MukulPdeSystemSolver<DIM>::UpdateCellData(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
{
    // Store the PDE solution in an accessible form
    ReplicatableVector solution_repl(this->mSolution);

    for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = rCellPopulation.Begin();
         cell_iter != rCellPopulation.End();
         ++cell_iter)
    {
        // The cells are not nodes of the mesh, so we must interpolate
        double solution_at_cell = 0.0;

        // Find the element in the FE mesh that contains this cell. CellElementMap has been updated so use this.
        unsigned elem_index = mCellPdeElementMap[*cell_iter];
        Element<DIM,DIM>* p_element = this->mpMesh->GetElement(elem_index);

        const ChastePoint<DIM>& node_location = rCellPopulation.GetLocationOfCellCentre(*cell_iter);

        c_vector<double,DIM+1> weights = p_element->CalculateInterpolationWeights(node_location);

        for (unsigned i=0; i<DIM+1; i++)
        {
            double nodal_value = solution_repl[p_element->GetNodeGlobalIndex(i)];
            solution_at_cell += nodal_value * weights(i);
        }

        cell_iter->GetCellData()->SetItem("bmp", solution_at_cell);
        cell_iter->GetCellData()->SetItem("nog", solution_at_cell);
    }
}

template<unsigned DIM>
void MukulPdeSystemSolver<DIM>::InitialiseCellPdeElementMap(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
{
    mCellPdeElementMap.clear();

    // Find the element of mpMesh that contains each cell and populate mCellPdeElementMap
    for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = rCellPopulation.Begin();
         cell_iter != rCellPopulation.End();
         ++cell_iter)
    {
        const ChastePoint<DIM>& r_position_of_cell = rCellPopulation.GetLocationOfCellCentre(*cell_iter);
        unsigned elem_index = this->mpMesh->GetContainingElementIndex(r_position_of_cell);
        mCellPdeElementMap[*cell_iter] = elem_index;
    }
}

template<unsigned DIM>
void MukulPdeSystemSolver<DIM>::UpdateCellPdeElementMap(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
{
    // Find the element of mpCoarsePdeMesh that contains each cell and populate mCellPdeElementMap
    for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = rCellPopulation.Begin();
         cell_iter != rCellPopulation.End();
         ++cell_iter)
    {
        const ChastePoint<DIM>& r_position_of_cell = rCellPopulation.GetLocationOfCellCentre(*cell_iter);
        unsigned elem_index = this->mpMesh->GetContainingElementIndexWithInitialGuess(r_position_of_cell, mCellPdeElementMap[*cell_iter]);
        mCellPdeElementMap[*cell_iter] = elem_index;
    }
}

template<unsigned DIM>
void MukulPdeSystemSolver<DIM>::UpdateAtEndOfOutputTimeStep(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
{
#ifdef CHASTE_VTK
    if (DIM > 1)
    {
        std::ostringstream time_string;
        time_string << SimulationTime::Instance()->GetTimeStepsElapsed();
        std::string results_file = "pde_results_bmp_" + time_string.str();
        VtkMeshWriter<DIM,DIM>* p_vtk_mesh_writer = new VtkMeshWriter<DIM,DIM>(mOutputDirectory, results_file, false);

        ReplicatableVector solution_repl(mSolution);
        std::vector<double> pde_solution;
        for (unsigned i=0; i<mpMesh->GetNumNodes(); i++)
        {
           pde_solution.push_back(solution_repl[i]);
        }

        p_vtk_mesh_writer->AddPointData("bmp", pde_solution);
        p_vtk_mesh_writer->WriteFilesUsingMesh(*mpMesh);

        delete p_vtk_mesh_writer;
    }
#endif //CHASTE_VTK
}

template<unsigned DIM>
void MukulPdeSystemSolver<DIM>::UpdateAtEndOfSolve(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
{
}

template<unsigned DIM>
void MukulPdeSystemSolver<DIM>::SetupLinearSystem(Vec currentSolution, bool computeMatrix)
{
    this->SetupGivenLinearSystem(currentSolution, computeMatrix, this->mpLinearSystem);
}

#include "SerializationExportWrapper.hpp"
TEMPLATED_CLASS_IS_ABSTRACT_1_UNSIGNED(MukulPdeSystemSolver)

#endif /*MUKULPDESYSTEMSOLVER_HPP_*/
