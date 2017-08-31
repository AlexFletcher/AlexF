
#ifndef STRETCHBASEDCELLCYCLEMODEL_HPP_
#define STRETCHBASEDCELLCYCLEMODEL_HPP_

#include "AbstractCellCycleModel.hpp"
#include "RandomNumberGenerator.hpp"

class StretchBasedCellCycleModel : public AbstractCellCycleModel
{
private:

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & archive, const unsigned int version)
    {
        archive & boost::serialization::base_object<AbstractCellCycleModel>(*this);
    }

protected:

    StretchBasedCellCycleModel(const StretchBasedCellCycleModel& rModel);

public:

    StretchBasedCellCycleModel();
    virtual bool ReadyToDivide();
    AbstractCellCycleModel* CreateCellCycleModel();
    void ResetForDivision();
    double GetAverageTransitCellCycleTime();
    double GetAverageStemCellCycleTime();
    double GetReferenceTargetArea();
    double GetTargetArea();
    void SetReferenceTargetArea(double referenceTargetArea);
    void SetMaxGrowthRate(double maxGrowthRate);
    double GetMaxGrowthRate();
    virtual void OutputCellCycleModelParameters(out_stream& rParamsFile);
};

#include "SerializationExportWrapper.hpp"
// Declare identifier for the serializer
CHASTE_CLASS_EXPORT(StretchBasedCellCycleModel)

#endif /* STRETCHBASEDCELLCYCLEMODEL_HPP_ */
