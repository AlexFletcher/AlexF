#ifndef SPHEROIDDATAWRITER_HPP_
#define SPHEROIDDATAWRITER_HPP_

#include "AbstractCellPopulationWriter.hpp"
#include "ChasteSerialization.hpp"
#include <boost/serialization/base_object.hpp>

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
class SpheroidDataWriter : public AbstractCellPopulationWriter<ELEMENT_DIM, SPACE_DIM>
{
private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & archive, const unsigned int version)
    {
        archive & boost::serialization::base_object<AbstractCellPopulationWriter<ELEMENT_DIM, SPACE_DIM> >(*this);
    }

public:

    SpheroidDataWriter();

    void VisitAnyPopulation(AbstractCellPopulation<SPACE_DIM, SPACE_DIM>* pCellPopulation);
    virtual void Visit(MeshBasedCellPopulation<ELEMENT_DIM, SPACE_DIM>* pCellPopulation);
    virtual void Visit(CaBasedCellPopulation<SPACE_DIM>* pCellPopulation);
    virtual void Visit(NodeBasedCellPopulation<SPACE_DIM>* pCellPopulation);
    virtual void Visit(PottsBasedCellPopulation<SPACE_DIM>* pCellPopulation);
    virtual void Visit(VertexBasedCellPopulation<SPACE_DIM>* pCellPopulation);
};

#include "SerializationExportWrapper.hpp"
// Declare identifier for the serializer
EXPORT_TEMPLATE_CLASS_ALL_DIMS(SpheroidDataWriter)

#endif /* SPHEROIDDATAWRITER_HPP_ */
