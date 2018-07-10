#include "SpheroidDataWriter.hpp"
#include "MeshBasedCellPopulation.hpp"
#include "CaBasedCellPopulation.hpp"
#include "NodeBasedCellPopulation.hpp"
#include "PottsBasedCellPopulation.hpp"
#include "VertexBasedCellPopulation.hpp"

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
SpheroidDataWriter<ELEMENT_DIM, SPACE_DIM>::SpheroidDataWriter()
    : AbstractCellPopulationWriter<ELEMENT_DIM, SPACE_DIM>("spheroiddata.dat")
{
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void SpheroidDataWriter<ELEMENT_DIM, SPACE_DIM>::VisitAnyPopulation(AbstractCellPopulation<SPACE_DIM, SPACE_DIM>* pCellPopulation)
{
    // Record the number of cells
    unsigned num_cells = pCellPopulation->GetNumRealCells();

    // Record the radius of gyration and maximum radius of the cell population
    c_vector<double, SPACE_DIM> centre = pCellPopulation->GetCentroidOfCellPopulation();
    double squared_radius_of_gyration = 0.0;
    double max_radius = -DBL_MAX;
    for (typename AbstractCellPopulation<SPACE_DIM, SPACE_DIM>::Iterator cell_iter = pCellPopulation->Begin();
         cell_iter != pCellPopulation->End();
         ++cell_iter)
    {
        double radius = norm_2(pCellPopulation->GetLocationOfCellCentre(*cell_iter) - centre);
        squared_radius_of_gyration += radius*radius / (double) num_cells;

        if (radius > max_radius)
        {
            max_radius = radius;
        }
    }
    double radius_of_gyration = sqrt(squared_radius_of_gyration);

    *this->mpOutStream << num_cells << "\t" << radius_of_gyration << "\t" << max_radius;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void SpheroidDataWriter<ELEMENT_DIM, SPACE_DIM>::Visit(MeshBasedCellPopulation<ELEMENT_DIM, SPACE_DIM>* pCellPopulation)
{
    // Record the number of cells
    unsigned num_cells = pCellPopulation->GetNumRealCells();

    // Record the radius of gyration and maximum radius of the cell population
    c_vector<double, SPACE_DIM> centre = pCellPopulation->GetCentroidOfCellPopulation();
    double squared_radius_of_gyration = 0.0;
    double max_radius = -DBL_MAX;
    for (typename AbstractCellPopulation<ELEMENT_DIM, SPACE_DIM>::Iterator cell_iter = pCellPopulation->Begin();
         cell_iter != pCellPopulation->End();
         ++cell_iter)
    {
        double radius = norm_2(pCellPopulation->GetLocationOfCellCentre(*cell_iter) - centre);
        squared_radius_of_gyration += radius*radius / (double) num_cells;

        if (radius > max_radius)
        {
            max_radius = radius;
        }
    }
    double radius_of_gyration = sqrt(squared_radius_of_gyration);

    *this->mpOutStream << num_cells << "\t" << radius_of_gyration << "\t" << max_radius;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void SpheroidDataWriter<ELEMENT_DIM, SPACE_DIM>::Visit(CaBasedCellPopulation<SPACE_DIM>* pCellPopulation)
{
    VisitAnyPopulation(pCellPopulation);
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void SpheroidDataWriter<ELEMENT_DIM, SPACE_DIM>::Visit(NodeBasedCellPopulation<SPACE_DIM>* pCellPopulation)
{
    VisitAnyPopulation(pCellPopulation);
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void SpheroidDataWriter<ELEMENT_DIM, SPACE_DIM>::Visit(PottsBasedCellPopulation<SPACE_DIM>* pCellPopulation)
{
    VisitAnyPopulation(pCellPopulation);
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void SpheroidDataWriter<ELEMENT_DIM, SPACE_DIM>::Visit(VertexBasedCellPopulation<SPACE_DIM>* pCellPopulation)
{
    VisitAnyPopulation(pCellPopulation);
}

// Explicit instantiation
template class SpheroidDataWriter<1,1>;
template class SpheroidDataWriter<1,2>;
template class SpheroidDataWriter<2,2>;
template class SpheroidDataWriter<1,3>;
template class SpheroidDataWriter<2,3>;
template class SpheroidDataWriter<3,3>;

#include "SerializationExportWrapperForCpp.hpp"
// Declare identifier for the serializer
EXPORT_TEMPLATE_CLASS_ALL_DIMS(SpheroidDataWriter)
