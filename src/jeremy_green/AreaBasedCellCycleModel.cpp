
#include "AreaBasedCellCycleModel.hpp"
#include "Exception.hpp"
#include "StemCellProliferativeType.hpp"
#include "TransitCellProliferativeType.hpp"
#include "DifferentiatedCellProliferativeType.hpp"

AreaBasedCellCycleModel::AreaBasedCellCycleModel()
    : AbstractCellCycleModel(),
      mReferenceTargetArea(1.0),
      mTargetArea(1.0),
      mMaxGrowthRate(1.0)
{
}

AreaBasedCellCycleModel::AreaBasedCellCycleModel(const AreaBasedCellCycleModel& rModel)
    : AbstractCellCycleModel(rModel),
      mReferenceTargetArea(1.0),
      mTargetArea(1.0),
      mMaxGrowthRate(1.0)
{
}

bool AreaBasedCellCycleModel::ReadyToDivide()
{
    if (!mReadyToDivide)
    {
        double cell_area = mpCell->GetCellData()->GetItem("volume");
        if (cell_area >= 2.0*mReferenceTargetArea)
        {
            mReadyToDivide = true;
        }

        double growth_rate = RandomNumberGenerator::Instance()->ranf()*mMaxGrowthRate;
        double dt = SimulationTime::Instance()->GetTimeStep();
        mTargetArea += growth_rate*dt;
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
}

double AreaBasedCellCycleModel::GetMaxGrowthRate()
{
    return mMaxGrowthRate;
}

void AreaBasedCellCycleModel::OutputCellCycleModelParameters(out_stream& rParamsFile)
{
    *rParamsFile << "\t\t\t<ReferenceTargetArea>" << mReferenceTargetArea << "</ReferenceTargetArea>\n";
    *rParamsFile << "\t\t\t<MaxGrowthRate>" << mMaxGrowthRate << "</MaxGrowthRate>\n";
    AbstractCellCycleModel::OutputCellCycleModelParameters(rParamsFile);
}

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
CHASTE_CLASS_EXPORT(AreaBasedCellCycleModel)
