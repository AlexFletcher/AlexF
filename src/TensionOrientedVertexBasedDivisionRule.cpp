#include "TensionOrientedVertexBasedDivisionRule.hpp"

template <unsigned SPACE_DIM>
c_vector<double, SPACE_DIM> TensionOrientedVertexBasedDivisionRule<SPACE_DIM>::CalculateCellDivisionVector(
    CellPtr pParentCell,
    VertexBasedCellPopulation<SPACE_DIM>& rCellPopulation)
{
    ///\todo implement tension oriented cell division rule
    c_vector<double, SPACE_DIM> random_vector;
    double random_angle = 2.0*M_PI*RandomNumberGenerator::Instance()->ranf();
    random_vector(0) = cos(random_angle);
    random_vector(1) = sin(random_angle);
    return random_vector;
}

// Explicit instantiation
template class TensionOrientedVertexBasedDivisionRule<1>;
template class TensionOrientedVertexBasedDivisionRule<2>;
template class TensionOrientedVertexBasedDivisionRule<3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(TensionOrientedVertexBasedDivisionRule)
