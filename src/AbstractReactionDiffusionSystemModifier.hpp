//
//#ifndef ABSTRACTREACTIONDIFFUSIONSYSTEMMODIFIER_HPP_
//#define ABSTRACTREACTIONDIFFUSIONSYSTEMMODIFIER_HPP_
//
//#include "ChasteSerialization.hpp"
//#include "ClassIsAbstract.hpp"
//#include <boost/shared_ptr.hpp>
//
//#include "AbstractCellBasedSimulationModifier.hpp"
//#include "TetrahedralMesh.hpp"
//#include "AbstractLinearPde.hpp"
//#include "AbstractBoundaryCondition.hpp"
//
///**
// * \todo Document class
// */
//template<unsigned DIM, unsigned PROBLEM_DIM>
//class AbstractReactionDiffusionSystemModifier : public AbstractCellBasedSimulationModifier<DIM,DIM>
//{
//private:
//
//    /** Needed for serialization. */
//    friend class boost::serialization::access;
//    /**
//     * Boost Serialization method for archiving/checkpointing.
//     * Archives the object and its member variables.
//     *
//     * @param archive  The boost archive.
//     * @param version  The current version of this class.
//     */
//    template<class Archive>
//    void serialize(Archive & archive, const unsigned int version)
//    {
//        archive & boost::serialization::base_object<AbstractCellBasedSimulationModifier<DIM,DIM> >(*this);
//        archive & mpReactionDiffusionSystem;
//        archive & mpBoundaryConditions;
//        archive & mDependentVariableNames;
//        archive & mpFeMesh;
//        // Archiving of mSolution is handled by the methods save/load_construct_data
//        archive & mOutputDirectory;
//        archive & mOutputGradients;
//        archive & mOutputSolutionAtPdeNodes;
//    }
//
//protected:
//
//    /**
//     * Shared pointer to a linear PDE object.
//     */
//    boost::shared_ptr<AbstractLinearParabolicPdeSystemForCoupledOdeSystem<DIM,DIM,PROBLEM_DIM> > mpReactionDiffusionSystem;
//
//    /**
//     * Shared pointer to a boundary conditions container object.
//     */
//    boost::shared_ptr<BoundaryConditionsContainer<DIM, DIM, PROBLEM_DIM> > mpBoundaryConditions;
//
//    /**
//     * For use in PDEs where we know what the quantities for which we are solving are called,
//     * e.g. oxygen concentration.
//     */
//    std::vector<std::string> mDependentVariableNames;
//
//    /** The solution to the PDE problem at the current time step. */
//    Vec mSolution;
//
//    /** Pointer to the finite element mesh on which to solve the PDE. */
//    TetrahedralMesh<DIM,DIM>* mpFeMesh;
//
//    /** Store the output directory name. */
//    std::string mOutputDirectory;
//
//    /** Whether or not to calculate and output the gradient of the solution. */
//    bool mOutputGradient;
//
//    /**
//     * Whether to output the PDE solution at each node of the FE mesh at output time steps.
//     * Defaults to false.
//     */
//    bool mOutputSolutionAtPdeNodes;
//
//    /** File that the values of the PDE solution are written out to. */
//    out_stream mpVizPdeSolutionResultsFile;
//
//    /**
//     * Whether to delete the finite element mesh when we are destroyed.
//     */
//    bool mDeleteFeMesh;
//
//public:
//
//    /**
//     * Constructor.
//     *
//     * @param pPde A shared pointer to a linear PDE object (defaults to NULL)
//     * @param pBoundaryCondition A shared pointer to an abstract boundary condition
//     *     (defaults to NULL, corresponding to a constant boundary condition with value zero)
//     * @param isNeumannBoundaryCondition Whether the boundary condition is Neumann (defaults to true)
//     * @param solution solution vector (defaults to NULL)
//     */
//    AbstractReactionDiffusionSystemModifier(
//        boost::shared_ptr<AbstractLinearParabolicPdeSystemForCoupledOdeSystem<DIM,DIM,PROBLEM_DIM> > pReactionDiffusionSystem=boost::shared_ptr<AbstractLinearParabolicPdeSystemForCoupledOdeSystem<DIM,DIM,PROBLEM_DIM> >(),
//        boost::shared_ptr<BoundaryConditionsContainer<DIM, DIM, PROBLEM_DIM> > pBoundaryCondition=boost::shared_ptr<BoundaryConditionsContainer<DIM, DIM, PROBLEM_DIM> >(),
//        Vec solution=nullptr);
//
//    /**
//     * Destructor.
//     */
//    virtual ~AbstractReactionDiffusionSystemModifier();
//
//    /**
//     * @return mpReactionDiffusionSystem
//     */
//    boost::shared_ptr<AbstractLinearParabolicPdeSystemForCoupledOdeSystem<DIM,DIM,PROBLEM_DIM> > GetReactionDiffusionSystem();
//
//    /**
//     * @return mpBoundaryConditions
//     */
//    boost::shared_ptr<BoundaryConditionsContainer<DIM, DIM, PROBLEM_DIM> > GetBoundaryConditions();
//
//    /**
//     * Set the names of the dependent variables.
//     *
//     * @param rName the name.
//     */
//    void SetDependentVariableNames(const std::vector<std::string>& rNames);
//
//    /**
//     * Get the names of the dependent variables.
//     *
//     * @return the name
//     */
//    std::vector<std::string>& rGetDependentVariableNames();
//
//    /**
//     * @return whether the PDE has an averaged source
//     * \todo edit?
//     */
//    bool HasAveragedSourcePde();
//
//    /**
//     * In the case where the PDE has an averaged source, set the source terms
//     * using the information in the given mesh.
//     *
//     * @param pMesh Pointer to a tetrahedral mesh
//     * @param pCellPdeElementMap map between cells and elements
//     *
//     *  \todo edit?
//     */
//    void SetUpSourceTermsForAveragedSourcePde(TetrahedralMesh<DIM,DIM>* pMesh, std::map<CellPtr, unsigned>* pCellPdeElementMap=nullptr);
//
//    /**
//     * @return mSolution.
//     */
//    Vec GetSolution();
//
//    /**
//     * @return mSolution (used in archiving)
//     */
//    Vec GetSolution() const;
//
//    /**
//     * @return mpFeMesh.
//     */
//    TetrahedralMesh<DIM,DIM>* GetFeMesh() const;
//
//    /**
//     * Overridden SetupSolve() method.
//     *
//     * Set mOutputDirectory and, if mOutputSolutionAtPdeNodes is set to true, open mpVizPdeSolutionResultsFile.
//     * This method is overridden in subclasses.
//     *
//     * @param rCellPopulation reference to the cell population
//     * @param outputDirectory the output directory, relative to where Chaste output is stored
//     */
//    virtual void SetupSolve(AbstractCellPopulation<DIM,DIM>& rCellPopulation, std::string outputDirectory);
//
//    /**
//     * Overridden UpdateAtEndOfTimeStep() method.
//     *
//     * As this method is pure virtual, it must be overridden in subclasses.
//     *
//     * @param rCellPopulation reference to the cell population
//     */
//    virtual void UpdateAtEndOfTimeStep(AbstractCellPopulation<DIM,DIM>& rCellPopulation);
//
//    /**
//     * Overridden UpdateAtEndOfOutputTimeStep() method,
//     * after UpdateAtEndOfTimeStep() has been called.
//     *
//     * Output the solution to the PDE at each cell to VTK and, if mOutputSolutionAtPdeNodes is set to true,
//     * output the solution to the PDE at each node of mpFeMesh to mpVizPdeSolutionResultsFile.
//     *
//     * @param rCellPopulation reference to the cell population
//     */
//    virtual void UpdateAtEndOfOutputTimeStep(AbstractCellPopulation<DIM,DIM>& rCellPopulation);
//
//    /**
//     * Overridden UpdateAtEndOfSolve() method.
//     *
//     * If mOutputSolutionAtPdeNodes is set to true, close mpVizPdeSolutionResultsFile.
//     *
//     * @param rCellPopulation reference to the cell population
//     */
//    virtual void UpdateAtEndOfSolve(AbstractCellPopulation<DIM,DIM>& rCellPopulation);
//
//    /**
//     * Set whether to calculate and save the gradient of the solution to CellData.
//     *
//     * @return mOutputGradient
//     */
//    bool GetOutputGradient();
//
//    /**
//     * Set whether to calculate and save the gradient of the solution to CellData.
//     *
//     * @param outputGradient whether to output the gradient
//     */
//    void SetOutputGradient(bool outputGradient);
//
//    /**
//     * Set mOutputSolutionAtPdeNodes.
//     *
//     * @param outputSolutionAtPdeNodes whether to output the PDE solution at each node of the FE mesh at output time steps
//     */
//    void SetOutputSolutionAtPdeNodes(bool outputSolutionAtPdeNodes);
//
//    /**
//     * Overridden OutputSimulationModifierParameters() method.
//     *
//     * Output any simulation modifier parameters to file.
//     *
//     * @param rParamsFile the file stream to which the parameters are output
//     */
//    void OutputSimulationModifierParameters(out_stream& rParamsFile);
//
//    /**
//     * Helper method to copy the PDE solution to CellData
//     *
//     * Here we need to interpolate from the FE mesh onto the cells.
//     *
//     * @param rCellPopulation reference to the cell population
//     */
//    void UpdateCellData(AbstractCellPopulation<DIM,DIM>& rCellPopulation);
//
//    /**
//     * Initialise mCellPdeElementMap.
//     *
//     * @param rCellPopulation reference to the cell population
//     */
//    void InitialiseCellPdeElementMap(AbstractCellPopulation<DIM,DIM>& rCellPopulation);
//
//    /**
//     * Update the mCellPdeElementMap
//     *
//     * This method should be called before sending the element map to a PDE class
//     * to ensure map is up to date.
//     *
//     * @param rCellPopulation reference to the cell population
//     */
//    void UpdateCellPdeElementMap(AbstractCellPopulation<DIM,DIM>& rCellPopulation);
//
////    /**
////     * Helper method to construct the boundary conditions container for the PDE.
////     *
////     * @param rCellPopulation reference to the cell population
////     *
////     * @return the full boundary conditions container
////     */
////    virtual std::shared_ptr<BoundaryConditionsContainer<DIM,DIM,1> > ConstructBoundaryConditionsContainer(AbstractCellPopulation<DIM,DIM>& rCellPopulation);
//
//    /**
//     * Helper method to initialise the PDE solution using the CellData.
//     *
//     * Here we assume a homogeneous initial consition.
//     *
//     * @param rCellPopulation reference to the cell population
//     */
//    void SetupInitialSolutionVector(AbstractCellPopulation<DIM,DIM>& rCellPopulation);
//};
//
//#include "SerializationExportWrapper.hpp"
//TEMPLATED_CLASS_IS_ABSTRACT_2_UNSIGNED(AbstractReactionDiffusionSystemModifier)
//
//#endif /*ABSTRACTREACTIONDIFFUSIONSYSTEMMODIFIER_HPP_*/
