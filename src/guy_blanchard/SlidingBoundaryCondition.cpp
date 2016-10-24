
#include "SlidingBoundaryCondition.hpp"

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
SlidingBoundaryCondition<ELEMENT_DIM,SPACE_DIM>::SlidingBoundaryCondition(AbstractCellPopulation<ELEMENT_DIM,SPACE_DIM>* pCellPopulation,
                                                    double yMin,
                                                    double yMax)
    : AbstractCellPopulationBoundaryCondition<ELEMENT_DIM,SPACE_DIM>(pCellPopulation),
      mYMin(yMin),
      mYMax(yMax)
{
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void SlidingBoundaryCondition<ELEMENT_DIM,SPACE_DIM>::ImposeBoundaryCondition(const std::map<Node<SPACE_DIM>*, c_vector<double, SPACE_DIM> >& rOldLocations)
{
    double epsilon = 1e-4;

    for (unsigned node_index=0; node_index<this->mpCellPopulation->GetNumNodes(); node_index++)
    {
        Node<SPACE_DIM>* p_node = this->mpCellPopulation->GetNode(node_index);

        if (p_node->IsBoundaryNode())
        {
            if ((fabs(p_node->rGetLocation()[1] - yMin) > epsilon)) && (fabs(p_node->rGetLocation()[1] - yMax) > epsilon))
            {
                ///\todo implement vertical sliding
            }
        }
        else
        {
            ///\todo implement horizontal sliding
        }
        ///\todo fix corner nodes
    }

//        unsigned node_index = this->mpCellPopulation->GetLocationIndexUsingCell(*cell_iter);
//        Node<SPACE_DIM>* p_node = this->mpCellPopulation->GetNode(node_index);
//
//        c_vector<double, SPACE_DIM> node_location = p_node->rGetLocation();
//
//        double signed_distance = inner_prod(node_location - mPointOnPlane, mNormalToPlane);
//        if (signed_distance > 0.0)
//        {
//            // For the closest point on the plane we travel from node_location the signed_distance in the direction of -mNormalToPlane
//            c_vector<double, SPACE_DIM> nearest_point;
//
//            nearest_point = node_location - signed_distance*mNormalToPlane;
//            p_node->rGetModifiableLocation() = nearest_point;
//        }
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
bool SlidingBoundaryCondition<ELEMENT_DIM,SPACE_DIM>::VerifyBoundaryCondition()
{
    return true;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void SlidingBoundaryCondition<ELEMENT_DIM,SPACE_DIM>::OutputCellPopulationBoundaryConditionParameters(out_stream& rParamsFile)
{
    *rParamsFile << "\t\t\t<YMin>" << mYMin << "</YMin>\n";
    *rParamsFile << "\t\t\t<YMax>" << mYMax << "</YMax>\n";

    // Call method on direct parent class
    AbstractCellPopulationBoundaryCondition<ELEMENT_DIM,SPACE_DIM>::OutputCellPopulationBoundaryConditionParameters(rParamsFile);
}

// Explicit instantiation
template class SlidingBoundaryCondition<1,1>;
template class SlidingBoundaryCondition<1,2>;
template class SlidingBoundaryCondition<2,2>;
template class SlidingBoundaryCondition<1,3>;
template class SlidingBoundaryCondition<2,3>;
template class SlidingBoundaryCondition<3,3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_ALL_DIMS(SlidingBoundaryCondition)
