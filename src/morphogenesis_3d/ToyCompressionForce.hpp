
#ifndef TOYCOMPRESSIONFORCE_HPP_
#define TOYCOMPRESSIONFORCE_HPP_

#include "ChasteSerialization.hpp"
#include <boost/serialization/base_object.hpp>
#include "Exception.hpp"
#include "AbstractForce.hpp"
#include "VertexBasedCellPopulation.hpp"
#include <iostream>

template<unsigned DIM>
class ToyCompressionForce  : public AbstractForce<DIM>
{
private:

    friend class boost::serialization::access;
    /**
     * Boost Serialization method for archiving/checkpointing.
     * Archives the object and its member variables.
     *
     * @param archive  The boost archive.
     * @param version  The current version of this class.
     */
    template<class Archive>
    void serialize(Archive & archive, const unsigned int version)
    {
        archive & boost::serialization::base_object<AbstractForce<DIM> >(*this);
    }

public:

    /**
     * Constructor.
     */
    ToyCompressionForce();

    /**
     * Destructor.
     */
    virtual ~ToyCompressionForce();

    /**
     * Overridden AddForceContribution() method.
     *
     * @param rCellPopulation reference to the cell population
     */
    virtual void AddForceContribution(AbstractCellPopulation<DIM>& rCellPopulation);

    /**
     * Overridden OutputForceParameters() method.
     *
     * @param rParamsFile the file stream to which the parameters are output
     */
    void OutputForceParameters(out_stream& rParamsFile);
};

#include "SerializationExportWrapper.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(ToyCompressionForce)

#endif /*TOYCOMPRESSIONFORCE_HPP_*/
