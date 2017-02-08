
#include "CellGenerationsWriter.hpp"
#include "AbstractCellPopulation.hpp"
#include "GammaDistributedGenerationalCellCycleModel.hpp"

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
CellGenerationsWriter<ELEMENT_DIM, SPACE_DIM>::CellGenerationsWriter()
    : AbstractCellWriter<ELEMENT_DIM, SPACE_DIM>("generations.dat")
{
    this->mVtkCellDataName = "Generation";
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
double CellGenerationsWriter<ELEMENT_DIM, SPACE_DIM>::GetCellDataForVtkOutput(CellPtr pCell, AbstractCellPopulation<ELEMENT_DIM, SPACE_DIM>* pCellPopulation)
{
    unsigned generation = (static_cast<GammaDistributedGenerationalCellCycleModel*>(pCell->GetCellCycleModel()))->GetGeneration();
    return (double) generation;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CellGenerationsWriter<ELEMENT_DIM, SPACE_DIM>::VisitCell(CellPtr pCell, AbstractCellPopulation<ELEMENT_DIM, SPACE_DIM>* pCellPopulation)
{
    // Don't both writing anything to file
}

// Explicit instantiation
template class CellGenerationsWriter<1,1>;
template class CellGenerationsWriter<1,2>;
template class CellGenerationsWriter<2,2>;
template class CellGenerationsWriter<1,3>;
template class CellGenerationsWriter<2,3>;
template class CellGenerationsWriter<3,3>;

#include "SerializationExportWrapperForCpp.hpp"
// Declare identifier for the serializer
EXPORT_TEMPLATE_CLASS_ALL_DIMS(CellGenerationsWriter)
