#ifndef CELLPACKINGDATAWRITER_HPP_
#define CELLPACKINGDATAWRITER_HPP_

#include "ChasteSerialization.hpp"
#include <boost/serialization/base_object.hpp>
#include "AbstractCellWriter.hpp"

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
class CellPackingDataWriter : public AbstractCellWriter<ELEMENT_DIM, SPACE_DIM>
{
private:

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & archive, const unsigned int version)
    {
        archive & boost::serialization::base_object<AbstractCellWriter<ELEMENT_DIM, SPACE_DIM> >(*this);
    }

public:

    CellPackingDataWriter();
    double GetCellDataForVtkOutput(CellPtr pCell, AbstractCellPopulation<ELEMENT_DIM, SPACE_DIM>* pCellPopulation);
    virtual void VisitCell(CellPtr pCell, AbstractCellPopulation<ELEMENT_DIM, SPACE_DIM>* pCellPopulation);
    void WriteTimeStamp();
    virtual void WriteNewline();

    /**
     * Helper function to determine the average area of cell neighbours for this cell
     *
     * @param pCell the cell to write
     * @param pCellPopulation a pointer to the cell population owning the cell.
     */
    double GetAverageCellAreaOfNeighbours(CellPtr pCell, AbstractCellPopulation<ELEMENT_DIM, SPACE_DIM>* pCellPopulation);

    /**
     * Helper function to determine the average number of neighbours for this cell
     *
     * @param pCell the cell to write
     * @param pCellPopulation a pointer to the cell population owning the cell.
     */
    double GetAverageNeighbourNumberOfNeighbours(CellPtr pCell, AbstractCellPopulation<ELEMENT_DIM, SPACE_DIM>* pCellPopulation);

    /**
     * Helper function to determine if any neighbours of the cell are on the boundary of the population.
     *
     * @param pCell the cell to write
     * @param pCellPopulation a pointer to the cell population owning the cell.
     */
    bool IsCellOnInnerBoundary(CellPtr pCell, AbstractCellPopulation<ELEMENT_DIM, SPACE_DIM>* pCellPopulation);
};

#include "SerializationExportWrapper.hpp"
EXPORT_TEMPLATE_CLASS_ALL_DIMS(CellPackingDataWriter)

#endif /*CELLPACKINGDATAWRITER_HPP_*/
