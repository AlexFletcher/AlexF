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
};

#include "SerializationExportWrapper.hpp"
EXPORT_TEMPLATE_CLASS_ALL_DIMS(CellPackingDataWriter)

#endif /*CELLPACKINGDATAWRITER_HPP_*/
