
#include "AreaBasedCellCycleModel.hpp"

AreaBasedCellCycleModel::AreaBasedCellCycleModel()
    : AbstractCellCycleModel(),
      mReferenceTargetArea(DOUBLE_UNSET),
      mTargetArea(DOUBLE_UNSET),
      mMaxGrowthRate(DOUBLE_UNSET),
      mGrowthRate(DOUBLE_UNSET)
{
}

AreaBasedCellCycleModel::AreaBasedCellCycleModel(const AreaBasedCellCycleModel& rModel)
    : AbstractCellCycleModel(rModel),
      mReferenceTargetArea(rModel.mReferenceTargetArea),
      mTargetArea(rModel.mTargetArea),
      mMaxGrowthRate(rModel.mMaxGrowthRate)
{
    mGrowthRate = RandomNumberGenerator::Instance()->ranf()*mMaxGrowthRate;
}

bool AreaBasedCellCycleModel::ReadyToDivide()
{
    assert(mReferenceTargetArea != DOUBLE_UNSET);
    assert(mMaxGrowthRate != DOUBLE_UNSET);
    assert(mGrowthRate != DOUBLE_UNSET);

    if (!mReadyToDivide)
    {
        double cell_area = mpCell->GetCellData()->GetItem("volume");
        if (cell_area >= 2.0*mReferenceTargetArea)
        {
            mReadyToDivide = true;
        }

        mTargetArea = mReferenceTargetArea + mGrowthRate*GetAge();
    }

    return mReadyToDivide;
}

AbstractCellCycleModel* AreaBasedCellCycleModel::CreateCellCycleModel()
{
    return new AreaBasedCellCycleModel(*this);
}

void AreaBasedCellCycleModel::ResetForDivision()
{
    mTargetArea = mReferenceTargetArea;
    AbstractCellCycleModel::ResetForDivision();
}

double AreaBasedCellCycleModel::GetAverageTransitCellCycleTime()
{
    return 0.0;
}

double AreaBasedCellCycleModel::GetAverageStemCellCycleTime()
{
    return 0.0;
}

double AreaBasedCellCycleModel::GetReferenceTargetArea()
{
    return mReferenceTargetArea;
}

double AreaBasedCellCycleModel::GetTargetArea()
{
    return mTargetArea;
}

void AreaBasedCellCycleModel::SetReferenceTargetArea(double referenceTargetArea)
{
    assert(referenceTargetArea >= 0.0);
    mReferenceTargetArea = referenceTargetArea;
}

void AreaBasedCellCycleModel::SetMaxGrowthRate(double maxGrowthRate)
{
    mMaxGrowthRate = maxGrowthRate;
    mGrowthRate = RandomNumberGenerator::Instance()->ranf()*mMaxGrowthRate;
}

double AreaBasedCellCycleModel::GetMaxGrowthRate()
{
    return mMaxGrowthRate;
}

void AreaBasedCellCycleModel::OutputCellCycleModelParameters(out_stream& rParamsFile)
{
    *rParamsFile << "\t\t\t<ReferenceTargetArea>" << mReferenceTargetArea << "</ReferenceTargetArea>\n";
    *rParamsFile << "\t\t\t<MaxGrowthRate>" << mMaxGrowthRate << "</MaxGrowthRate>\n";
    *rParamsFile << "\t\t\t<GrowthRate>" << mGrowthRate << "</GrowthRate>\n";
    AbstractCellCycleModel::OutputCellCycleModelParameters(rParamsFile);
}

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
CHASTE_CLASS_EXPORT(AreaBasedCellCycleModel)
