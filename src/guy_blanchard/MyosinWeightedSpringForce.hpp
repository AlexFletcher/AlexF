
#ifndef MYOSINWEIGHTEDSPRINGFORCE_HPP_
#define MYOSINWEIGHTEDSPRINGFORCE_HPP_

#include "AbstractTwoBodyInteractionForce.hpp"

#include "ChasteSerialization.hpp"
#include <boost/serialization/base_object.hpp>

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM=ELEMENT_DIM>
class MyosinWeightedSpringForce : public AbstractTwoBodyInteractionForce<ELEMENT_DIM, SPACE_DIM>
{
    friend class TestForces;

private:

    /** Needed for serialization. */
    friend class boost::serialization::access;
    /**
     * Archive the object and its member variables.
     *
     * @param archive the archive
     * @param version the current version of this class
     */
    template<class Archive>
    void serialize(Archive & archive, const unsigned int version)
    {
        archive & boost::serialization::base_object<AbstractTwoBodyInteractionForce<ELEMENT_DIM, SPACE_DIM> >(*this);
        archive & mMyosinSpringStiffness;
        archive & mMyosinSpringNaturalLength;
        archive & mNonMyosinSpringStiffness;
        archive & mNonMyosinSpringNaturalLength;
    }

protected:

    double mMyosinSpringStiffness;
    double mMyosinSpringNaturalLength;
    double mNonMyosinSpringStiffness;
    double mNonMyosinSpringNaturalLength;

public:

    MyosinWeightedSpringForce();
    virtual ~MyosinWeightedSpringForce();

    c_vector<double, SPACE_DIM> CalculateForceBetweenNodes(unsigned nodeAGlobalIndex,
                                                           unsigned nodeBGlobalIndex,
                                                           AbstractCellPopulation<ELEMENT_DIM,SPACE_DIM>& rCellPopulation);

    double GetMyosinSpringStiffness();
    double GetMyosinSpringNaturalLength();
    double GetNonMyosinSpringStiffness();
    double GetNonMyosinSpringNaturalLength();

    void SetMyosinSpringStiffness(double myosinSpringstiffness);
    void SetMyosinSpringNaturalLength(double myosinSpringNaturalLength);
    void SetNonMyosinSpringStiffness(double nonMyosinSpringstiffness);
    void SetNonMyosinSpringNaturalLength(double nonMyosinSpringnaturalLength);

    virtual void OutputForceParameters(out_stream& rParamsFile);
};

#include "SerializationExportWrapper.hpp"
EXPORT_TEMPLATE_CLASS_ALL_DIMS(MyosinWeightedSpringForce)

#endif /*MYOSINWEIGHTEDSPRINGFORCE_HPP_*/
