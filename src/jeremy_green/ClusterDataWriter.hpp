
#ifndef CLUSTERDATAWRITER_HPP_
#define CLUSTERDATAWRITER_HPP_

#include "AbstractCellPopulationWriter.hpp"
#include "ChasteSerialization.hpp"
#include <boost/serialization/base_object.hpp>

/**
 *\todo Document class and methods
 */
template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
class ClusterDataWriter : public AbstractCellPopulationWriter<ELEMENT_DIM, SPACE_DIM>
{
private:
    /** Needed for serialization. */
    friend class boost::serialization::access;
    /**
     * Serialize the object and its member variables.
     *
     * @param archive the archive
     * @param version the current version of this class
     */
    template<class Archive>
    void serialize(Archive & archive, const unsigned int version)
    {
        archive & boost::serialization::base_object<AbstractCellPopulationWriter<ELEMENT_DIM, SPACE_DIM> >(*this);
    }

public:

    /**
     * Default constructor.
     */
    ClusterDataWriter();

    virtual void Visit(MeshBasedCellPopulation<ELEMENT_DIM, SPACE_DIM>* pCellPopulation)
    {
    }

    virtual void Visit(CaBasedCellPopulation<SPACE_DIM>* pCellPopulation)
    {
    }

    virtual void Visit(NodeBasedCellPopulation<SPACE_DIM>* pCellPopulation)
    {
    }

    virtual void Visit(PottsBasedCellPopulation<SPACE_DIM>* pCellPopulation)
    {}

    virtual void Visit(VertexBasedCellPopulation<SPACE_DIM>* pCellPopulation);
};

#include "SerializationExportWrapper.hpp"
// Declare identifier for the serializer
EXPORT_TEMPLATE_CLASS_ALL_DIMS(ClusterDataWriter)

#endif /* CLUSTERDATAWRITER_HPP_ */
