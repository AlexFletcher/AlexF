
#ifndef APICALEDGESFORCE_HPP_
#define APICALEDGESFORCE_HPP_

#include "ChasteSerialization.hpp"
#include <boost/serialization/base_object.hpp>
#include "Exception.hpp"
#include "AbstractForce.hpp"
#include "VertexBasedCellPopulation.hpp"
#include <iostream>

template<unsigned DIM>
class ApicalEdgesForce  : public AbstractForce<DIM>
{
private:

    /**
     * Force coefficient, assumed to be the same constant for all apical edges.
     * Initialised to the defaults value 1.0 in the constructor.
     */
    double mLambda;

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
        archive & mLambda;
    }

public:

    /**
     * Constructor.
     */
    ApicalEdgesForce();

    /**
     * Destructor.
     */
    virtual ~ApicalEdgesForce();

    /**
     * Set the value of the force coefficient, assumed to be the same constant for all apical edges.
     *
     * @param lambda the value of mLambda
     */
    void SetLambda(double lambda);

    /**
     * @return the value of the force coefficient, assumed to be the same constant for all apical edges.
     */
    double GetLambda();

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
EXPORT_TEMPLATE_CLASS_SAME_DIMS(ApicalEdgesForce)

#endif /*APICALEDGESFORCE_HPP_*/
