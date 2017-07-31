#include "CircularBoundaryCondition.hpp"
#include "NodeBasedCellPopulation.hpp"

CircularBoundaryCondition::CircularBoundaryCondition(AbstractCellPopulation<2>* pCellPopulation,
													 c_vector<double,2> centre,
													 double radius)
    : AbstractCellPopulationBoundaryCondition<2>(pCellPopulation),
      mCentreOfCircle(centre),
      mRadiusOfCircle(radius)
{
    assert(mRadiusOfCircle > 0.0);
}

const c_vector<double,2>& CircularBoundaryCondition::rGetCentreOfCircle() const
{
    return mCentreOfCircle;
}

double CircularBoundaryCondition::GetRadiusOfCircle() const
{
    return mRadiusOfCircle;
}

void CircularBoundaryCondition::ImposeBoundaryCondition(const std::map<Node<2>*, c_vector<double,2> >& rOldLocations)
{
    assert(dynamic_cast<AbstractCentreBasedCellPopulation<1>*>(this->mpCellPopulation) != nullptr);

    for (AbstractCellPopulation<2>::Iterator cell_iter = this->mpCellPopulation->Begin();
         cell_iter != this->mpCellPopulation->End();
         ++cell_iter)
    {
        c_vector<double,2> cell_location = this->mpCellPopulation->GetLocationOfCellCentre(*cell_iter);
        double radius = norm_2(cell_location - mCentreOfCircle);

        if (radius - mRadiusOfCircle > DBL_EPSILON)
        {
            unsigned node_index = this->mpCellPopulation->GetLocationIndexUsingCell(*cell_iter);
            Node<2>* p_node = this->mpCellPopulation->GetNode(node_index);
            p_node->rGetModifiableLocation() = mCentreOfCircle + mRadiusOfCircle*(cell_location - mCentreOfCircle)/radius;
        }
    }
}

bool CircularBoundaryCondition::VerifyBoundaryCondition()
{
    bool condition_satisfied = true;

    assert(dynamic_cast<AbstractCentreBasedCellPopulation<1>*>(this->mpCellPopulation) != nullptr);

    for (AbstractCellPopulation<2>::Iterator cell_iter = this->mpCellPopulation->Begin();
         cell_iter != this->mpCellPopulation->End();
         ++cell_iter)
    {
        c_vector<double,2> cell_location = this->mpCellPopulation->GetLocationOfCellCentre(*cell_iter);
        double radius = norm_2(cell_location - mCentreOfCircle);

        if (radius - mRadiusOfCircle > DBL_EPSILON)
        {
            condition_satisfied = false;
            break;
        }
    }

    return condition_satisfied;
}

void CircularBoundaryCondition::OutputCellPopulationBoundaryConditionParameters(out_stream& rParamsFile)
{
    *rParamsFile << "\t\t\t<CentreOfCircle>" << mCentreOfCircle[0] << "," << mCentreOfCircle[1] << "</CentreOfCircle>\n";
    *rParamsFile << "\t\t\t<RadiusOfCircle>" << mRadiusOfCircle << "</RadiusOfCircle>\n";
    AbstractCellPopulationBoundaryCondition<2>::OutputCellPopulationBoundaryConditionParameters(rParamsFile);
}

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
CHASTE_CLASS_EXPORT(CircularBoundaryCondition)
