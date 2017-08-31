
#ifndef FARHADIFARFORCEWITHTIMEDEPENDENTCOEFFICIENTS_HPP_
#define FARHADIFARFORCEWITHTIMEDEPENDENTCOEFFICIENTS_HPP_

#include "ChasteSerialization.hpp"
#include <boost/serialization/base_object.hpp>
#include "Exception.hpp"

#include "FarhadifarForce.hpp"
#include "VertexBasedCellPopulation.hpp"

#include <iostream>

template<unsigned DIM>
class FarhadifarForceWithTimeDependentCoefficients : public FarhadifarForce<DIM>
{
friend class TestForces;

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
        archive & boost::serialization::base_object<FarhadifarForce<DIM> >(*this);
    }

public:

    /**
     * Constructor.
     */
    FarhadifarForceWithTimeDependentCoefficients();

    /**
     * Destructor.
     */
    virtual ~FarhadifarForceWithTimeDependentCoefficients();

    /**
     * @return mAreaElasticityParameter
     */
    virtual double GetAreaElasticityParameter();

    /**
     * @return mPerimeterContractilityParameter
     */
    virtual double GetPerimeterContractilityParameter();

    /**
     * Overridden OutputForceParameters() method.
     *
     * @param rParamsFile the file stream to which the parameters are output
     */
    void OutputForceParameters(out_stream& rParamsFile);
};

#include "SerializationExportWrapper.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(FarhadifarForceWithTimeDependentCoefficients)

#endif /*FARHADIFARFORCEWITHTIMEDEPENDENTCOEFFICIENTS_HPP_*/
