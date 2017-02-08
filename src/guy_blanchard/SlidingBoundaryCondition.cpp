
#include "SlidingBoundaryCondition.hpp"

SlidingBoundaryCondition::SlidingBoundaryCondition(AbstractCellPopulation<2,2>* pCellPopulation,
                                                   double xMin,
                                                   double yMin,
                                                   double xMax,
                                                   double yMax)
    : AbstractCellPopulationBoundaryCondition<2,2>(pCellPopulation),
      mXMin(xMin),
      mYMin(yMin),
      mXMax(xMax),
      mYMax(yMax)
{
}

void SlidingBoundaryCondition::ImposeBoundaryCondition(const std::map<Node<2>*, c_vector<double, 2> >& rOldLocations)
{
    double epsilon = 1e-1;

    // Loop over every node
    for (unsigned node_index=0; node_index<this->mpCellPopulation->GetNumNodes(); node_index++)
    {
        Node<2>* p_node = this->mpCellPopulation->GetNode(node_index);

        if (p_node->IsBoundaryNode())
        {
            c_vector<double, 2> old_node_location;
            old_node_location = rOldLocations.find(p_node)->second;

            // If the node lies on the top or bottom boundary, then revert its y coordinate
            if ((fabs(p_node->rGetLocation()[1] - mYMin) < epsilon) || (fabs(p_node->rGetLocation()[1] - mYMax) < epsilon))
            {
                p_node->rGetModifiableLocation()[1] = old_node_location[1];
            }

            // If the node lies on the left or right boundary, then revert its x coordinate
            if ((p_node->rGetLocation()[0] - mXMin < 0.5+epsilon) || (p_node->rGetLocation()[0] - mXMax > -(0.5+epsilon)))
            {
                p_node->rGetModifiableLocation()[0] = old_node_location[0];
            }
        }
    }
}

bool SlidingBoundaryCondition::VerifyBoundaryCondition()
{
    return true;
}

void SlidingBoundaryCondition::OutputCellPopulationBoundaryConditionParameters(out_stream& rParamsFile)
{
    *rParamsFile << "\t\t\t<XMin>" << mXMin << "</YMin>\n";
    *rParamsFile << "\t\t\t<YMin>" << mYMin << "</YMin>\n";
    *rParamsFile << "\t\t\t<XMax>" << mXMax << "</XMax>\n";
    *rParamsFile << "\t\t\t<YMax>" << mYMax << "</YMax>\n";

    // Call method on direct parent class
    AbstractCellPopulationBoundaryCondition<2,2>::OutputCellPopulationBoundaryConditionParameters(rParamsFile);
}

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
CHASTE_CLASS_EXPORT(SlidingBoundaryCondition)
