
#include "StretchBasedCellCycleModel.hpp"
#include "RandomNumberGenerator.hpp"

StretchBasedCellCycleModel::StretchBasedCellCycleModel()
    : AbstractCellCycleModel()
{
}

StretchBasedCellCycleModel::StretchBasedCellCycleModel(const StretchBasedCellCycleModel& rModel)
    : AbstractCellCycleModel(rModel)
{
}

bool StretchBasedCellCycleModel::ReadyToDivide()
{
	if (!mReadyToDivide)
	{
		double cell_stretch = mpCell->GetCellData()->GetItem("stretch");
		double r = RandomNumberGenerator::Instance()->ranf();
		if (r < cell_stretch*SimulationTime::Instance()->GetTimeStep()/100.0)
		{
			mReadyToDivide = true;
		}
	}

    return mReadyToDivide;
}

AbstractCellCycleModel* StretchBasedCellCycleModel::CreateCellCycleModel()
{
    return new StretchBasedCellCycleModel(*this);
}

void StretchBasedCellCycleModel::ResetForDivision()
{
    AbstractCellCycleModel::ResetForDivision();
}

double StretchBasedCellCycleModel::GetAverageTransitCellCycleTime()
{
    return 0.0;
}

double StretchBasedCellCycleModel::GetAverageStemCellCycleTime()
{
    return 0.0;
}

void StretchBasedCellCycleModel::OutputCellCycleModelParameters(out_stream& rParamsFile)
{
    AbstractCellCycleModel::OutputCellCycleModelParameters(rParamsFile);
}

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
CHASTE_CLASS_EXPORT(StretchBasedCellCycleModel)
