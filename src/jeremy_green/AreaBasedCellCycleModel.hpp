
#ifndef AREABASEDCELLCYCLEMODEL_HPP_
#define AREABASEDCELLCYCLEMODEL_HPP_

#include "AbstractCellCycleModel.hpp"
#include "RandomNumberGenerator.hpp"

/*
 * \todo Document this class
 */
class AreaBasedCellCycleModel : public AbstractCellCycleModel
{
private:

    double mReferenceTargetArea;
    double mTargetArea;
    double mMaxGrowthRate;
    double mGrowthRate;

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & archive, const unsigned int version)
    {
        archive & boost::serialization::base_object<AbstractCellCycleModel>(*this);
        archive & mReferenceTargetArea;
        archive & mTargetArea;
        archive & mMaxGrowthRate;
        archive & mGrowthRate;
    }

protected:

    AreaBasedCellCycleModel(const AreaBasedCellCycleModel& rModel);

public:

    AreaBasedCellCycleModel();
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
CHASTE_CLASS_EXPORT(AreaBasedCellCycleModel)

#endif /* AREABASEDCELLCYCLEMODEL_HPP_ */
