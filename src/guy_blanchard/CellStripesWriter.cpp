#include "CellStripesWriter.hpp"

#include "AbstractCellPopulation.hpp"
#include "ApoptoticCellProperty.hpp"
#include "CellLabel.hpp"
#include "WildTypeCellMutationState.hpp"

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
CellStripesWriter<ELEMENT_DIM, SPACE_DIM>::CellStripesWriter()
    : AbstractCellWriter<ELEMENT_DIM, SPACE_DIM>("results.vizstripes")
{
    this->mVtkCellDataName = "Stripes";
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
double CellStripesWriter<ELEMENT_DIM, SPACE_DIM>::GetCellDataForVtkOutput(CellPtr pCell, AbstractCellPopulation<ELEMENT_DIM, SPACE_DIM>* pCellPopulation)
{
    double stripe_identity = (double)(pCell->GetCellData()->GetItem("stripe"));
    return stripe_identity;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CellStripesWriter<ELEMENT_DIM, SPACE_DIM>::VisitCell(CellPtr pCell, AbstractCellPopulation<ELEMENT_DIM, SPACE_DIM>* pCellPopulation)
{
    double stripe_identity = (double)(pCell->GetCellData()->GetItem("stripe"));
    *this->mpOutStream << stripe_identity << " ";
}

// Explicit instantiation
template class CellStripesWriter<1,1>;
template class CellStripesWriter<1,2>;
template class CellStripesWriter<2,2>;
template class CellStripesWriter<1,3>;
template class CellStripesWriter<2,3>;
template class CellStripesWriter<3,3>;

#include "SerializationExportWrapperForCpp.hpp"
// Declare identifier for the serializer
EXPORT_TEMPLATE_CLASS_ALL_DIMS(CellStripesWriter)
