
#ifndef STRETCHTRACKINGMODIFIER_HPP_
#define STRETCHTRACKINGMODIFIER_HPP_

#include "ChasteSerialization.hpp"
#include <boost/serialization/base_object.hpp>

#include "AbstractCellBasedSimulationModifier.hpp"

template<unsigned DIM>
class StretchTrackingModifier : public AbstractCellBasedSimulationModifier<DIM,DIM>
{
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & archive, const unsigned int version)
    {
        archive & boost::serialization::base_object<AbstractCellBasedSimulationModifier<DIM,DIM> >(*this);
    }

public:

    StretchTrackingModifier();
    virtual ~StretchTrackingModifier();
    virtual void UpdateAtEndOfTimeStep(AbstractCellPopulation<DIM,DIM>& rCellPopulation);
    virtual void SetupSolve(AbstractCellPopulation<DIM,DIM>& rCellPopulation, std::string outputDirectory);
    void UpdateCellData(AbstractCellPopulation<DIM,DIM>& rCellPopulation);
    void OutputSimulationModifierParameters(out_stream& rParamsFile);
};

#include "SerializationExportWrapper.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(StretchTrackingModifier)

#endif /*STRETCHTRACKINGMODIFIER_HPP_*/
