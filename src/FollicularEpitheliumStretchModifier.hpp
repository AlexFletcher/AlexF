#ifndef FOLLICULAREPITHELIUMSTRETCHMODIFIER_HPP_
#define FOLLICULAREPITHELIUMSTRETCHMODIFIER_HPP_

#include "ChasteSerialization.hpp"
#include <boost/serialization/base_object.hpp>

#include "AbstractCellBasedSimulationModifier.hpp"

///\todo allow for non-uniform stretching

template<unsigned DIM>
class FollicularEpitheliumStretchModifier : public AbstractCellBasedSimulationModifier<DIM,DIM>
{
private:

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & archive, const unsigned int version)
    {
        archive & boost::serialization::base_object<AbstractCellBasedSimulationModifier<DIM,DIM> >(*this);
        archive & mApplyExtrinsicPullToAllNodes;
        archive & mPinAnteriorMostCells;
        archive & mSpeed;
        archive & mIncreaseStretchOverTime;
    }

    bool mApplyExtrinsicPullToAllNodes;
    bool mPinAnteriorMostCells;
    double mSpeed;
    bool mIncreaseStretchOverTime;

public:

    FollicularEpitheliumStretchModifier();
    virtual ~FollicularEpitheliumStretchModifier();
    virtual void UpdateAtEndOfTimeStep(AbstractCellPopulation<DIM,DIM>& rCellPopulation);
    virtual void SetupSolve(AbstractCellPopulation<DIM,DIM>& rCellPopulation, std::string outputDirectory);
    void ApplyExtrinsicPullToAllNodes(bool applyExtrinsicPullToAllNodes);
    void PinAnteriorMostCells(bool pinAnteriorMostCells);
    void SetSpeed(double speed);
    void IncreaseStretchOverTime(bool increaseStretchOverTime);
    void OutputSimulationModifierParameters(out_stream& rParamsFile);
};

#include "SerializationExportWrapper.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(FollicularEpitheliumStretchModifier)

#endif /*FOLLICULAREPITHELIUMSTRETCHMODIFIER_HPP_*/
