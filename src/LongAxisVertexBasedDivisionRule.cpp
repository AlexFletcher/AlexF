
#include "LongAxisVertexBasedDivisionRule.hpp"

template <unsigned SPACE_DIM>
c_vector<double, SPACE_DIM> LongAxisVertexBasedDivisionRule<SPACE_DIM>::CalculateCellDivisionVector(
    CellPtr pParentCell,
    VertexBasedCellPopulation<SPACE_DIM>& rCellPopulation)
{
    unsigned elem_index = rCellPopulation.GetLocationIndexUsingCell(pParentCell);
    c_vector<double, SPACE_DIM> short_axis = rCellPopulation.rGetMesh().GetShortAxisOfElement(elem_index);

    c_vector<double, SPACE_DIM> long_axis;
    long_axis[0] = -short_axis[1];
    long_axis[1] = short_axis[0];

    return long_axis;
}

// Explicit instantiation
template class LongAxisVertexBasedDivisionRule<1>;
template class LongAxisVertexBasedDivisionRule<2>;
template class LongAxisVertexBasedDivisionRule<3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(LongAxisVertexBasedDivisionRule)
