#include "MyosinWeightedSpringForce.hpp"
#include "CellLabel.hpp"

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
MyosinWeightedSpringForce<ELEMENT_DIM,SPACE_DIM>::MyosinWeightedSpringForce()
   : AbstractTwoBodyInteractionForce<ELEMENT_DIM,SPACE_DIM>(),
     mMyosinSpringStiffness(1.0),
     mMyosinSpringNaturalLength(1.0),
     mNonMyosinSpringStiffness(1.0),
     mNonMyosinSpringNaturalLength(1.0)
{
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
MyosinWeightedSpringForce<ELEMENT_DIM,SPACE_DIM>::~MyosinWeightedSpringForce()
{
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
c_vector<double, SPACE_DIM> MyosinWeightedSpringForce<ELEMENT_DIM,SPACE_DIM>::CalculateForceBetweenNodes(unsigned nodeAGlobalIndex,
                                                                                                         unsigned nodeBGlobalIndex,
                                                                                                         AbstractCellPopulation<ELEMENT_DIM,SPACE_DIM>& rCellPopulation)
{
    Node<SPACE_DIM>* p_node_a = rCellPopulation.GetNode(nodeAGlobalIndex);
    Node<SPACE_DIM>* p_node_b = rCellPopulation.GetNode(nodeBGlobalIndex);

    // Get the node locations
    const c_vector<double, SPACE_DIM>& r_node_a_location = p_node_a->rGetLocation();
    const c_vector<double, SPACE_DIM>& r_node_b_location = p_node_b->rGetLocation();

    // Get the unit vector parallel to the line joining the two nodes
    c_vector<double, SPACE_DIM> unit_difference = r_node_b_location - r_node_a_location;
    double distance_between_nodes = norm_2(unit_difference);
    unit_difference /= distance_between_nodes;

    // If neither cell is labelled, then use mMyosinSpringStiffness and mMyosinSpringNaturalLength
    double spring_stiffness = mNonMyosinSpringStiffness;
    double spring_rest_length = mNonMyosinSpringNaturalLength;

    bool cell_A_labelled = rCellPopulation.GetCellUsingLocationIndex(nodeAGlobalIndex)->template HasCellProperty<CellLabel>();
    bool cell_B_labelled = rCellPopulation.GetCellUsingLocationIndex(nodeBGlobalIndex)->template HasCellProperty<CellLabel>();

    if (cell_A_labelled && cell_B_labelled)
    {
        // If both cells are labelled, then use mMyosinSpringStiffness and mMyosinSpringNaturalLength
        spring_stiffness = mMyosinSpringStiffness;
        spring_rest_length = mMyosinSpringNaturalLength;
    }
    else if (cell_A_labelled || cell_B_labelled)
    {
        // If one cell is labelled, then use the geometric average
        spring_stiffness = sqrt(mMyosinSpringStiffness*mNonMyosinSpringStiffness);
        spring_rest_length = sqrt(mMyosinSpringNaturalLength*mNonMyosinSpringNaturalLength);
    }

    return spring_stiffness * unit_difference * (distance_between_nodes - spring_rest_length);
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
double MyosinWeightedSpringForce<ELEMENT_DIM,SPACE_DIM>::GetMyosinSpringStiffness()
{
    return mMyosinSpringStiffness;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>double
MyosinWeightedSpringForce<ELEMENT_DIM,SPACE_DIM>::GetMyosinSpringNaturalLength()
{
    return mMyosinSpringNaturalLength;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
double MyosinWeightedSpringForce<ELEMENT_DIM,SPACE_DIM>::GetNonMyosinSpringStiffness()
{
    return mNonMyosinSpringStiffness;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
double MyosinWeightedSpringForce<ELEMENT_DIM,SPACE_DIM>::GetNonMyosinSpringNaturalLength()
{
    return mNonMyosinSpringNaturalLength;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void MyosinWeightedSpringForce<ELEMENT_DIM,SPACE_DIM>::SetMyosinSpringStiffness(double myosinSpringstiffness)
{
    mMyosinSpringStiffness = myosinSpringstiffness;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void MyosinWeightedSpringForce<ELEMENT_DIM,SPACE_DIM>::SetMyosinSpringNaturalLength(double myosinSpringNaturalLength)
{
    mMyosinSpringNaturalLength = myosinSpringNaturalLength;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void MyosinWeightedSpringForce<ELEMENT_DIM,SPACE_DIM>::SetNonMyosinSpringStiffness(double nonMyosinSpringstiffness)
{
    mNonMyosinSpringStiffness = nonMyosinSpringstiffness;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void MyosinWeightedSpringForce<ELEMENT_DIM,SPACE_DIM>::SetNonMyosinSpringNaturalLength(double nonMyosinSpringNaturalLength)
{
    mNonMyosinSpringNaturalLength = nonMyosinSpringNaturalLength;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void MyosinWeightedSpringForce<ELEMENT_DIM,SPACE_DIM>::OutputForceParameters(out_stream& rParamsFile)
{
    *rParamsFile << "\t\t\t<MyosinSpringStiffness>" << mMyosinSpringStiffness << "</MyosinSpringStiffness>\n";
    *rParamsFile << "\t\t\t<MyosinSpringNaturalLength>" << mMyosinSpringNaturalLength << "</MyosinSpringNaturalLength>\n";
    *rParamsFile << "\t\t\t<NonMyosinSpringStiffness>" << mNonMyosinSpringStiffness << "</NonMyosinSpringStiffness>\n";
    *rParamsFile << "\t\t\t<NonMyosinSpringNaturalLength>" << mNonMyosinSpringNaturalLength << "</NonMyosinSpringNaturalLength>\n";

    AbstractTwoBodyInteractionForce<ELEMENT_DIM,SPACE_DIM>::OutputForceParameters(rParamsFile);
}

// Explicit instantiation
template class MyosinWeightedSpringForce<1,1>;
template class MyosinWeightedSpringForce<1,2>;
template class MyosinWeightedSpringForce<2,2>;
template class MyosinWeightedSpringForce<1,3>;
template class MyosinWeightedSpringForce<2,3>;
template class MyosinWeightedSpringForce<3,3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_ALL_DIMS(MyosinWeightedSpringForce)
