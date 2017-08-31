
#include <cassert>
#include "CellPackingDataWriter.hpp"
#include "AbstractCellPopulation.hpp"
#include "Cell.hpp"
#include "MutableVertexMesh.hpp"

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
CellPackingDataWriter<ELEMENT_DIM, SPACE_DIM>::CellPackingDataWriter()
    : AbstractCellWriter<ELEMENT_DIM, SPACE_DIM>("CellPackingData.txt")
{
    this->mVtkCellDataName = "CellPackingData";
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
double CellPackingDataWriter<ELEMENT_DIM, SPACE_DIM>::GetCellDataForVtkOutput(CellPtr pCell, AbstractCellPopulation<ELEMENT_DIM, SPACE_DIM>* pCellPopulation)
{
    return 1.0;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CellPackingDataWriter<ELEMENT_DIM, SPACE_DIM>::VisitCell(CellPtr pCell, AbstractCellPopulation<ELEMENT_DIM, SPACE_DIM>* pCellPopulation)
{
    // Assume we are using a VertexBasedCellPopulation
    MutableVertexMesh<ELEMENT_DIM, SPACE_DIM>* p_mesh = static_cast<MutableVertexMesh<ELEMENT_DIM, SPACE_DIM>* >(&(pCellPopulation->rGetMesh()));

    unsigned cell_id = pCell->GetCellId();
    unsigned location_index = pCellPopulation->GetLocationIndexUsingCell(pCell);
    bool on_boundary = p_mesh->GetElement(location_index)->IsElementOnBoundary();
    unsigned num_edges = p_mesh->GetElement(location_index)->GetNumNodes();
    double cell_area = p_mesh->GetVolumeOfElement(location_index);
    double cell_perimeter = p_mesh->GetSurfaceAreaOfElement(location_index);
    double shape_factor = p_mesh->GetElongationShapeFactorOfElement(location_index);

    *this->mpOutStream << cell_id << " "
                       << location_index << " "
                       << on_boundary << " "
                       << num_edges << " "
                       << cell_area << " "
                       << cell_perimeter << " "
                       << shape_factor << " ";

    c_vector<double, SPACE_DIM> centre_location = pCellPopulation->GetLocationOfCellCentre(pCell);
    for (unsigned i=0; i<SPACE_DIM; i++)
    {
        *this->mpOutStream << centre_location[i]  << " ";
    }
}

// Explicit instantiation
template class CellPackingDataWriter<1,1>;
template class CellPackingDataWriter<1,2>;
template class CellPackingDataWriter<2,2>;
template class CellPackingDataWriter<1,3>;
template class CellPackingDataWriter<2,3>;
template class CellPackingDataWriter<3,3>;

#include "SerializationExportWrapperForCpp.hpp"
// Declare identifier for the serializer
EXPORT_TEMPLATE_CLASS_ALL_DIMS(CellPackingDataWriter)
