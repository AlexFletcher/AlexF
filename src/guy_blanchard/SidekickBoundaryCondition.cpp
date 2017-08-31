
#include "SidekickBoundaryCondition.hpp"

template<unsigned DIM>
SidekickBoundaryCondition<DIM>::SidekickBoundaryCondition(AbstractCellPopulation<DIM>* pCellPopulation)
    : AbstractCellPopulationBoundaryCondition<DIM>(pCellPopulation)
{
}

template<unsigned DIM>
void SidekickBoundaryCondition<DIM>::ImposeBoundaryCondition(const std::map<Node<DIM>*, c_vector<double, DIM> >& rOldLocations)
{
    double epsilon = 1e-1;

    // Loop over every node
    for (unsigned node_index=0; node_index<this->mpCellPopulation->GetNumNodes(); node_index++)
    {
        Node<DIM>* p_node = this->mpCellPopulation->GetNode(node_index);

        if (p_node->IsBoundaryNode())
        {
            c_vector<double, DIM> old_node_location;
            old_node_location = rOldLocations.find(p_node)->second;

            // If the node lies on the left or right boundary, then revert its x coordinate
            if (p_node->rGetLocation()[0] < 0.8+epsilon)
            {
                p_node->rGetModifiableLocation()[0] = old_node_location[0];
            }
        }
    }}

template<unsigned DIM>
bool SidekickBoundaryCondition<DIM>::VerifyBoundaryCondition()
{
    bool condition_satisfied = true;
//
//    // Iterate over the cell population
//    for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = this->mpCellPopulation->Begin();
//         cell_iter != this->mpCellPopulation->End();
//         ++cell_iter)
//    {
//        // Find the radial distance between this cell and the surface of the sphere
//        c_vector<double,DIM> cell_location = this->mpCellPopulation->GetLocationOfCellCentre(*cell_iter);
//        double radius = norm_2(cell_location - mCentreOfSphere);
//
//        // If the cell is too far from the surface of the sphere...
//        if (fabs(radius - mRadiusOfSphere) > mMaximumDistance)
//        {
//            // ...then the boundary condition is not satisfied
//            condition_satisfied = false;
//            break;
//        }
//    }
    return condition_satisfied;
}

template<unsigned DIM>
void SidekickBoundaryCondition<DIM>::OutputCellPopulationBoundaryConditionParameters(out_stream& rParamsFile)
{
    AbstractCellPopulationBoundaryCondition<DIM>::OutputCellPopulationBoundaryConditionParameters(rParamsFile);
}

// Explicit instantiation
template class SidekickBoundaryCondition<1>;
template class SidekickBoundaryCondition<2>;
template class SidekickBoundaryCondition<3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(SidekickBoundaryCondition)
