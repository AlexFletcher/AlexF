
#include "MeshBasedCellPopulationWithoutRemeshing.hpp"

template<unsigned DIM>
MeshBasedCellPopulationWithoutRemeshing<DIM>::MeshBasedCellPopulationWithoutRemeshing(MutableMesh<DIM, DIM>& rMesh,
                                                                                      std::vector<CellPtr>& rCells,
                                                                                      const std::vector<unsigned> locationIndices,
                                                                                      bool deleteMesh,
                                                                                      bool validate)
    : MeshBasedCellPopulation<DIM,DIM>(rMesh, rCells, locationIndices, deleteMesh, validate)
{
}

template<unsigned DIM>
MeshBasedCellPopulationWithoutRemeshing<DIM>::MeshBasedCellPopulationWithoutRemeshing(MutableMesh<DIM, DIM>& rMesh)
    : MeshBasedCellPopulation<DIM,DIM>(rMesh)
{
}

template<unsigned DIM>
MeshBasedCellPopulationWithoutRemeshing<DIM>::~MeshBasedCellPopulationWithoutRemeshing()
{
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void MeshBasedCellPopulationWithoutRemeshing<ELEMENT_DIM,SPACE_DIM>::Update(bool hasHadBirthsOrDeaths)
{
    // Prevent any cell rearrangements
}

// Explicit instantiation
template class MeshBasedCellPopulationWithoutRemeshing<1>;
template class MeshBasedCellPopulationWithoutRemeshing<2>;
template class MeshBasedCellPopulationWithoutRemeshing<3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(MeshBasedCellPopulationWithoutRemeshing)
