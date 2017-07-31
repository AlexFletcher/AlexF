
#include "CellPackingDataWriter.hpp"
#include "AbstractCellPopulation.hpp"
#include "Cell.hpp"
#include "CellLabel.hpp"
#include "MutableVertexMesh.hpp"

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/ref.hpp>
#include <boost/bind.hpp>
using namespace boost::accumulators;

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
CellPackingDataWriter<ELEMENT_DIM, SPACE_DIM>::CellPackingDataWriter()
    : AbstractCellWriter<ELEMENT_DIM, SPACE_DIM>("CellPackingData.txt")
{
    this->mVtkCellDataName = "CellPackingData";
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
double CellPackingDataWriter<ELEMENT_DIM, SPACE_DIM>::GetCellDataForVtkOutput(CellPtr pCell, AbstractCellPopulation<ELEMENT_DIM, SPACE_DIM>* pCellPopulation)
{
    return 2.0;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
double CellPackingDataWriter<ELEMENT_DIM, SPACE_DIM>::GetAverageCellAreaOfNeighbours(CellPtr pCell, AbstractCellPopulation<ELEMENT_DIM, SPACE_DIM>* pCellPopulation)
{
    unsigned location_index = pCellPopulation->GetLocationIndexUsingCell(pCell);

    MutableVertexMesh<ELEMENT_DIM, SPACE_DIM>* p_mesh = static_cast<MutableVertexMesh<ELEMENT_DIM, SPACE_DIM>* >(&(pCellPopulation->rGetMesh()));
    std::set<unsigned> indices_of_neighbour_elements = p_mesh->GetNeighbouringElementIndices(location_index);

    accumulator_set<double, features<tag::mean > > area_accumulator;

    for (std::set<unsigned>::iterator this_iter = indices_of_neighbour_elements.begin();
         this_iter != indices_of_neighbour_elements.end();
         this_iter++)
    {
        double this_area = p_mesh->GetVolumeOfElement(*this_iter);
        area_accumulator(this_area);
    }

    return mean(area_accumulator);
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
double CellPackingDataWriter<ELEMENT_DIM, SPACE_DIM>::GetAverageNeighbourNumberOfNeighbours(CellPtr pCell, AbstractCellPopulation<ELEMENT_DIM, SPACE_DIM>* pCellPopulation)
{
    unsigned location_index = pCellPopulation->GetLocationIndexUsingCell(pCell);

    MutableVertexMesh<ELEMENT_DIM, SPACE_DIM>* p_mesh = static_cast<MutableVertexMesh<ELEMENT_DIM, SPACE_DIM>* >(&(pCellPopulation->rGetMesh()));
    std::set<unsigned> indices_of_neighbour_elements = p_mesh->GetNeighbouringElementIndices(location_index);

    accumulator_set<double, features<tag::mean> > neighbour_number_accumulator;

    for (std::set<unsigned>::iterator this_iter = indices_of_neighbour_elements.begin();
         this_iter != indices_of_neighbour_elements.end();
         this_iter++)
    {
        double this_neighbour_number = p_mesh->GetElement(*this_iter)->GetNumNodes();
        neighbour_number_accumulator(this_neighbour_number);
    }

    return mean(neighbour_number_accumulator);
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
bool CellPackingDataWriter<ELEMENT_DIM, SPACE_DIM>::IsCellOnInnerBoundary(CellPtr pCell, AbstractCellPopulation<ELEMENT_DIM, SPACE_DIM>* pCellPopulation)
{
    unsigned location_index = pCellPopulation->GetLocationIndexUsingCell(pCell);

    MutableVertexMesh<ELEMENT_DIM, SPACE_DIM>* p_mesh = static_cast<MutableVertexMesh<ELEMENT_DIM, SPACE_DIM>* >(&(pCellPopulation->rGetMesh()));
    std::set<unsigned> indices_of_neighbour_elements = p_mesh->GetNeighbouringElementIndices(location_index);

    bool is_on_inner_boundary = false;

    for (std::set<unsigned>::iterator this_iter = indices_of_neighbour_elements.begin();
         this_iter != indices_of_neighbour_elements.end();
         this_iter++)
    {
        if (p_mesh->GetElement(*this_iter)->IsElementOnBoundary())
        {
            is_on_inner_boundary = true;
            break;
        }
    }

    return is_on_inner_boundary;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CellPackingDataWriter<ELEMENT_DIM, SPACE_DIM>::VisitCell(CellPtr pCell, AbstractCellPopulation<ELEMENT_DIM, SPACE_DIM>* pCellPopulation)
{
    unsigned location_index = pCellPopulation->GetLocationIndexUsingCell(pCell);
    unsigned cell_id = pCell->GetCellId();

    // Note that the line below will only return something sensible if using a VertexBasedCellPopulation
    MutableVertexMesh<ELEMENT_DIM, SPACE_DIM>* p_mesh = static_cast<MutableVertexMesh<ELEMENT_DIM, SPACE_DIM>* >(&(pCellPopulation->rGetMesh()));

    unsigned num_edges = p_mesh->GetElement(location_index)->GetNumNodes();
    double cell_area = p_mesh->GetVolumeOfElement(location_index);
    *this->mpOutStream << SimulationTime::Instance()->GetTime() << " " << location_index << " " << cell_id << " " << cell_type << " " << num_edges << " " << cell_area << " ";

    c_vector<double, SPACE_DIM> centre_location = pCellPopulation->GetLocationOfCellCentre(pCell);
    for (unsigned i=0; i<SPACE_DIM; i++)
    {
        *this->mpOutStream << " " << centre_location[i];
    }

    *this->mpOutStream << " " << p_mesh->GetElement(location_index)->IsElementOnBoundary();
    *this->mpOutStream << " " << p_mesh->GetSurfaceAreaOfElement(location_index);
    *this->mpOutStream << " " << p_mesh->GetElongationShapeFactorOfElement(location_index);
    *this->mpOutStream << " " << this->IsCellOnInnerBoundary( pCell, pCellPopulation);
    *this->mpOutStream << " " << this->GetAverageNeighbourNumberOfNeighbours( pCell, pCellPopulation);
    *this->mpOutStream << " " << this->GetAverageCellAreaOfNeighbours( pCell, pCellPopulation);
    *this->mpOutStream << "\n";
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CellPackingDataWriter<ELEMENT_DIM, SPACE_DIM>::WriteTimeStamp()
{
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CellPackingDataWriter<ELEMENT_DIM, SPACE_DIM>::WriteNewline()
{
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
