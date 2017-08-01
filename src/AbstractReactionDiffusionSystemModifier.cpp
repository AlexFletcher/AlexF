//
//#include "AbstractReactionDiffusionSystemModifier.hpp"
//#include "VtkMeshWriter.hpp"
//#include "ReplicatableVector.hpp"
//#include "AveragedSourceEllipticPde.hpp"
//#include "AveragedSourceParabolicPde.hpp"
//
//template<unsigned DIM>
//AbstractReactionDiffusionSystemModifier<DIM>::AbstractReactionDiffusionSystemModifier(
//    boost::shared_ptr<AbstractLinearParabolicPdeSystemForCoupledOdeSystem<DIM,DIM,PROBLEM_DIM> > pReactionDiffusionSystem,
//    boost::shared_ptr<BoundaryConditionsContainer<DIM, DIM, PROBLEM_DIM> > pBoundaryCondition,
//    Vec solution)
//    : AbstractCellBasedSimulationModifier<DIM>(),
//      mpReactionDiffusionSystem(pReactionDiffusionSystem),
//      mpBoundaryConditions(pBoundaryConditions),
//      mSolution(nullptr),
//      mOutputDirectory(""),
//      mOutputGradient(false),
//      mOutputSolutionAtPdeNodes(false),
//      mDeleteFeMesh(false)
//{
//    if (solution)
//    {
//        mSolution = solution;
//    }
//}
//
//template<unsigned DIM>
//AbstractReactionDiffusionSystemModifier<DIM>::~AbstractReactionDiffusionSystemModifier()
//{
//    if (mDeleteFeMesh and mpFeMesh!=nullptr)
//    {
//        delete mpFeMesh;
//    }
//    if (mSolution)
//    {
//        PetscTools::Destroy(mSolution);
//    }
//}
//
//template<unsigned DIM>
//boost::shared_ptr<AbstractLinearParabolicPdeSystemForCoupledOdeSystem<DIM,DIM,PROBLEM_DIM> > AbstractReactionDiffusionSystemModifier<DIM>::GetReactionDiffusionSystem()
//{
//    return mpReactionDiffusionSystem;
//}
//
//template<unsigned DIM>
//boost::shared_ptr<BoundaryConditionsContainer<DIM, DIM, PROBLEM_DIM> > AbstractReactionDiffusionSystemModifier<DIM>::GetBoundaryConditions()
//{
//    return mpBoundaryConditions;
//}
//
/////\todo remove?
//template<unsigned DIM>
//bool AbstractReactionDiffusionSystemModifier<DIM>::IsNeumannBoundaryCondition()
//{
//    return mIsNeumannBoundaryCondition;
//}
//
//template<unsigned DIM>
//void AbstractReactionDiffusionSystemModifier<DIM>::SetDependentVariableNames(const std::vector<std::string>& rName)
//{
//    mDependentVariableNames = rNames;
//}
//
//template<unsigned DIM>
//std::vector<std::string>& AbstractReactionDiffusionSystemModifier<DIM>::rGetDependentVariableNames()
//{
//    return mDependentVariableNames;
//}
//
/////\todo edit?
//template<unsigned DIM>
//bool AbstractReactionDiffusionSystemModifier<DIM>::HasAveragedSourcePde()
//{
//    return ((boost::dynamic_pointer_cast<AveragedSourceEllipticPde<DIM> >(mpPde) != nullptr) ||
//            (boost::dynamic_pointer_cast<AveragedSourceParabolicPde<DIM> >(mpPde) != nullptr));
//}
//
/////\todo edit?
//template<unsigned DIM>
//void AbstractReactionDiffusionSystemModifier<DIM>::SetUpSourceTermsForAveragedSourcePde(TetrahedralMesh<DIM,DIM>* pMesh, std::map<CellPtr, unsigned>* pCellPdeElementMap)
//{
//    assert(HasAveragedSourcePde());
//    if (boost::dynamic_pointer_cast<AveragedSourceEllipticPde<DIM> >(mpPde) != nullptr)
//    {
//        boost::static_pointer_cast<AveragedSourceEllipticPde<DIM> >(mpPde)->SetupSourceTerms(*pMesh, pCellPdeElementMap);
//    }
//    else if (boost::dynamic_pointer_cast<AveragedSourceParabolicPde<DIM> >(mpPde) != nullptr)
//    {
//        boost::static_pointer_cast<AveragedSourceParabolicPde<DIM> >(mpPde)->SetupSourceTerms(*pMesh, pCellPdeElementMap);
//    }
//}
//
//template<unsigned DIM>
//Vec AbstractReactionDiffusionSystemModifier<DIM>::GetSolution()
//{
//    return mSolution;
//}
//
//template<unsigned DIM>
//Vec AbstractReactionDiffusionSystemModifier<DIM>::GetSolution() const
//{
//    return mSolution;
//}
//
//template<unsigned DIM>
//TetrahedralMesh<DIM,DIM>* AbstractReactionDiffusionSystemModifier<DIM>::GetFeMesh() const
//{
//    return mpFeMesh;
//}
//
//template<unsigned DIM>
//void AbstractReactionDiffusionSystemModifier<DIM>::SetupSolve(AbstractCellPopulation<DIM,DIM>& rCellPopulation, std::string outputDirectory)
//{
//    // Cache the output directory
//    mOutputDirectory = outputDirectory;
//
//    if (mOutputSolutionAtPdeNodes)
//    {
//        if (PetscTools::AmMaster())
//        {
//            OutputFileHandler output_file_handler(outputDirectory+"/", false);
//            mpVizPdeSolutionResultsFile = output_file_handler.OpenOutputFile("results.vizpdesolution");
//        }
//    }
//
//    InitialiseCellPdeElementMap(rCellPopulation);
//
//    // Copy the cell data to mSolution (this is the initial condition)
//    SetupInitialSolutionVector(rCellPopulation);
//
//    // Output the initial conditions on FeMesh
//    UpdateAtEndOfOutputTimeStep(rCellPopulation);
//}
//
//template<unsigned DIM>
//void AbstractReactionDiffusionSystemModifier<DIM>::UpdateAtEndOfOutputTimeStep(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
//{
//    if (mOutputSolutionAtPdeNodes)
//    {
//        if (PetscTools::AmMaster())
//        {
//            (*mpVizPdeSolutionResultsFile) << SimulationTime::Instance()->GetTime() << "\t";
//
//            assert(mpFeMesh != nullptr);
//
//            for (unsigned i=0; i<mDependentVariableNames.size(); i++)
//            {
//				assert(mDependentVariableNames[i] != "");
//
//				for (unsigned i=0; i<mpFeMesh->GetNumNodes(); i++)
//				{
//					(*mpVizPdeSolutionResultsFile) << i << " ";
//					const c_vector<double,DIM>& r_location = mpFeMesh->GetNode(i)->rGetLocation();
//					for (unsigned k=0; k<DIM; k++)
//					{
//						(*mpVizPdeSolutionResultsFile) << r_location[k] << " ";
//					}
//
//					///\todo work out the structure of mSolution...
//					assert(mSolution != nullptr);
//					ReplicatableVector solution_repl(mSolution);
//					(*mpVizPdeSolutionResultsFile) << solution_repl[i] << " ";
//				}
//            }
//
//            (*mpVizPdeSolutionResultsFile) << "\n";
//        }
//    }
//#ifdef CHASTE_VTK
//    if (DIM > 1)
//    {
//        std::ostringstream time_string;
//        time_string << SimulationTime::Instance()->GetTimeStepsElapsed();
//        std::string results_file = "pde_results_" + mDependentVariableName + "_" + time_string.str();
//        VtkMeshWriter<DIM,DIM>* p_vtk_mesh_writer = new VtkMeshWriter<DIM,DIM>(mOutputDirectory, results_file, false);
//
//        for (unsigned i=0; i<mDependentVariablesNames.size(); i++)
//        {
//			ReplicatableVector solution_repl(mSolution);
//			std::vector<double> pde_solution;
//			for (unsigned i=0; i<mpFeMesh->GetNumNodes(); i++)
//			{
//			   pde_solution.push_back(solution_repl[i]);
//			}
//
//			p_vtk_mesh_writer->AddPointData(mDependentVariableNames[i], pde_solution);
//        }
//
//        p_vtk_mesh_writer->WriteFilesUsingMesh(*mpFeMesh);
//        delete p_vtk_mesh_writer;
//    }
//#endif //CHASTE_VTK
//}
//
//template<unsigned DIM>
//void AbstractReactionDiffusionSystemModifier<DIM>::UpdateAtEndOfSolve(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
//{
//    if (mOutputSolutionAtPdeNodes)
//    {
//        if (PetscTools::AmMaster())
//        {
//            mpVizPdeSolutionResultsFile->close();
//        }
//    }
//}
//
//template<unsigned DIM>
//bool AbstractReactionDiffusionSystemModifier<DIM>::GetOutputGradient()
//{
//    return mOutputGradient;
//}
//
//template<unsigned DIM>
//void AbstractReactionDiffusionSystemModifier<DIM>::SetOutputGradient(bool outputGradient)
//{
//    mOutputGradient = outputGradient;
//}
//
//template<unsigned DIM>
//void AbstractReactionDiffusionSystemModifier<DIM>::SetOutputSolutionAtPdeNodes(bool outputSolutionAtPdeNodes)
//{
//    mOutputSolutionAtPdeNodes = outputSolutionAtPdeNodes;
//}
//
//template<unsigned DIM>
//void AbstractReactionDiffusionSystemModifier<DIM>::OutputSimulationModifierParameters(out_stream& rParamsFile)
//{
//    AbstractCellBasedSimulationModifier<DIM>::OutputSimulationModifierParameters(rParamsFile);
//}
//
//template<unsigned DIM>
//void AbstractReactionDiffusionSystemModifier<DIM>::UpdateCellData(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
//{
//	///\todo work out structure of mSolution...
//    // Store the PDE solution in an accessible form
//    ReplicatableVector solution_repl(mSolution);
//
//    for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = rCellPopulation.Begin();
//         cell_iter != rCellPopulation.End();
//         ++cell_iter)
//    {
//        // The cells are not nodes of the mesh, so we must interpolate
//        double solution_at_cell = 0.0;
//
//        // Find the element in the FE mesh that contains this cell. CellElementMap has been updated so use this.
//        unsigned elem_index = mCellPdeElementMap[*cell_iter];
//        Element<DIM,DIM>* p_element = mpFeMesh->GetElement(elem_index);
//
//        const ChastePoint<DIM>& node_location = rCellPopulation.GetLocationOfCellCentre(*cell_iter);
//
//        c_vector<double,DIM+1> weights = p_element->CalculateInterpolationWeights(node_location);
//
//        for (unsigned i=0; i<DIM+1; i++)
//        {
//            double nodal_value = solution_repl[p_element->GetNodeGlobalIndex(i)];
//            solution_at_cell += nodal_value * weights(i);
//        }
//
//        ///\todo fix this
//        for (unsigned i=0; i<mDependentVariableNames.size(); i++)
//        {
//            cell_iter->GetCellData()->SetItem(mDependentVariableNames[i], solution_at_cell);
//        }
//
//        if (mOutputGradient)
//        {
//            // Now calculate the gradient of the solution and store this in CellVecData
//            c_vector<double, DIM> solution_gradient = zero_vector<double>(DIM);
//
//            // Calculate the basis functions at any point (e.g. zero) in the element
//            c_matrix<double, DIM, DIM> jacobian, inverse_jacobian;
//            double jacobian_det;
//            mpFeMesh->GetInverseJacobianForElement(elem_index, jacobian, jacobian_det, inverse_jacobian);
//            const ChastePoint<DIM> zero_point;
//            c_matrix<double, DIM, DIM+1> grad_phi;
//            LinearBasisFunction<DIM>::ComputeTransformedBasisFunctionDerivatives(zero_point, inverse_jacobian, grad_phi);
//
//            for (unsigned node_index=0; node_index<DIM+1; node_index++)
//            {
//                double nodal_value = solution_repl[p_element->GetNodeGlobalIndex(node_index)];
//
//                for (unsigned j=0; j<DIM; j++)
//                {
//                    solution_gradient(j) += nodal_value* grad_phi(j, node_index);
//                }
//            }
//
//            ///\todo fix this with [0] below
//            switch (DIM)
//            {
//                case 1:
//                    cell_iter->GetCellData()->SetItem(mDependentVariableNames[0]+"_grad_x", solution_gradient(0));
//                    break;
//                case 2:
//                    cell_iter->GetCellData()->SetItem(mDependentVariableNames[0]+"_grad_x", solution_gradient(0));
//                    cell_iter->GetCellData()->SetItem(mDependentVariableNames[0]+"_grad_y", solution_gradient(1));
//                    break;
//                case 3:
//                    cell_iter->GetCellData()->SetItem(mDependentVariableNames[0]+"_grad_x", solution_gradient(0));
//                    cell_iter->GetCellData()->SetItem(mDependentVariableNames[0]+"_grad_y", solution_gradient(1));
//                    cell_iter->GetCellData()->SetItem(mDependentVariableNames[0]+"_grad_z", solution_gradient(2));
//                    break;
//                default:
//                    NEVER_REACHED;
//            }
//        }
//    }
//}
//
//template<unsigned DIM>
//void AbstractReactionDiffusionSystemModifier<DIM>::InitialiseCellPdeElementMap(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
//{
//    mCellPdeElementMap.clear();
//
//    // Find the element of mpFeMesh that contains each cell and populate mCellPdeElementMap
//    for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = rCellPopulation.Begin();
//         cell_iter != rCellPopulation.End();
//         ++cell_iter)
//    {
//        const ChastePoint<DIM>& r_position_of_cell = rCellPopulation.GetLocationOfCellCentre(*cell_iter);
//        unsigned elem_index = mpFeMesh->GetContainingElementIndex(r_position_of_cell);
//        mCellPdeElementMap[*cell_iter] = elem_index;
//    }
//}
//
//template<unsigned DIM>
//void AbstractReactionDiffusionSystemModifier<DIM>::UpdateCellPdeElementMap(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
//{
//    // Find the element of mpCoarsePdeMesh that contains each cell and populate mCellPdeElementMap
//    for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = rCellPopulation.Begin();
//         cell_iter != rCellPopulation.End();
//         ++cell_iter)
//    {
//        const ChastePoint<DIM>& r_position_of_cell = rCellPopulation.GetLocationOfCellCentre(*cell_iter);
//        unsigned elem_index = mpFeMesh->GetContainingElementIndexWithInitialGuess(r_position_of_cell, mCellPdeElementMap[*cell_iter]);
//        mCellPdeElementMap[*cell_iter] = elem_index;
//    }
//}
//
//template<unsigned DIM>
//void AbstractReactionDiffusionSystemModifier<DIM>::UpdateAtEndOfTimeStep(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
//{
//    // Set up boundary conditions
//    std::shared_ptr<BoundaryConditionsContainer<DIM,DIM,1> > p_bcc = ConstructBoundaryConditionsContainer(rCellPopulation);
//
//    UpdateCellPdeElementMap(rCellPopulation);
//
//    // When using a PDE mesh which doesn't coincide with the cells, we must set up the source terms before solving the PDE.
//    // Pass in already updated CellPdeElementMap to speed up finding cells.
//    SetUpSourceTermsForAveragedSourcePde(mpFeMesh, &mCellPdeElementMap);
//
//    // Use SimpleLinearParabolicSolver as averaged Source PDE
//    SimpleLinearParabolicSolver<DIM,DIM> solver(mpFeMesh,
//                                                boost::static_pointer_cast<AbstractLinearParabolicPde<DIM,DIM> >(this->GetPde()).get(),
//                                                p_bcc.get());
//
//    ///\todo Investigate more than one PDE time step per spatial step
//    SimulationTime* p_simulation_time = SimulationTime::Instance();
//    double current_time = p_simulation_time->GetTime();
//    double dt = p_simulation_time->GetTimeStep();
//    solver.SetTimes(current_time,current_time + dt);
//    solver.SetTimeStep(dt);
//
//    // Use previous solution as the initial condition
//    Vec previous_solution = this->mSolution;
//    solver.SetInitialCondition(previous_solution);
//
//    // Note that the linear solver creates a vector, so we have to keep a handle on the old one
//    // in order to destroy it
//    mSolution = solver.Solve();
//    PetscTools::Destroy(previous_solution);
//    >UpdateCellData(rCellPopulation);
//}
//
////template<unsigned DIM>
////std::shared_ptr<BoundaryConditionsContainer<DIM,DIM,1> > AbstractReactionDiffusionSystemModifier<DIM>::ConstructBoundaryConditionsContainer(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
////{
////    std::shared_ptr<BoundaryConditionsContainer<DIM,DIM,1> > p_bcc(new BoundaryConditionsContainer<DIM,DIM,1>(false));
////
////    if (!this->mSetBcsOnBoxBoundary)
////    {
////        EXCEPTION("Boundary conditions cannot yet be set on the cell population boundary for a ParabolicBoxDomainPdeModifier");
////    }
////    else // Apply BC at boundary nodes of box domain FE mesh
////    {
////        if (this->IsNeumannBoundaryCondition())
////        {
////            // Impose any Neumann boundary conditions
////            for (typename TetrahedralMesh<DIM,DIM>::BoundaryElementIterator elem_iter = this->mpFeMesh->GetBoundaryElementIteratorBegin();
////                 elem_iter != this->mpFeMesh->GetBoundaryElementIteratorEnd();
////                 ++elem_iter)
////            {
////                p_bcc->AddNeumannBoundaryCondition(*elem_iter, this->mpBoundaryCondition.get());
////            }
////        }
////        else
////        {
////            // Impose any Dirichlet boundary conditions
////            for (typename TetrahedralMesh<DIM,DIM>::BoundaryNodeIterator node_iter = this->mpFeMesh->GetBoundaryNodeIteratorBegin();
////                 node_iter != this->mpFeMesh->GetBoundaryNodeIteratorEnd();
////                 ++node_iter)
////            {
////                p_bcc->AddDirichletBoundaryCondition(*node_iter, this->mpBoundaryCondition.get());
////            }
////        }
////    }
////
////    return p_bcc;
////}
//
//template<unsigned DIM>
//void AbstractReactionDiffusionSystemModifier<DIM>::SetupInitialSolutionVector(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
//{
//    // Specify homogeneous initial conditions based upon the values stored in CellData.
//    // Note need all the CellDataValues to be the same.
//
//    double initial_condition = rCellPopulation.Begin()->GetCellData()->GetItem(mDependentVariableName);
//
//    for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = rCellPopulation.Begin();
//         cell_iter != rCellPopulation.End();
//         ++cell_iter)
//    {
//        double initial_condition_at_cell = cell_iter->GetCellData()->GetItem(mDependentVariableName);
//        UNUSED_OPT(initial_condition_at_cell);
//        assert(fabs(initial_condition_at_cell - initial_condition)<1e-12);
//    }
//
//    // Initialise mSolution
//    mSolution = PetscTools::CreateAndSetVec(mpFeMesh->GetNumNodes(), initial_condition);
//}
//
//// Explicit instantiation
//template class AbstractReactionDiffusionSystemModifier<1>;
//template class AbstractReactionDiffusionSystemModifier<2>;
//template class AbstractReactionDiffusionSystemModifier<3>;
