
#include "GammaDistributedGenerationalCellCycleModel.hpp"
#include "Exception.hpp"
#include "StemCellProliferativeType.hpp"
#include "TransitCellProliferativeType.hpp"
#include "DifferentiatedCellProliferativeType.hpp"

GammaDistributedGenerationalCellCycleModel::GammaDistributedGenerationalCellCycleModel()
    : AbstractSimpleCellCycleModel(),
      mShape(DOUBLE_UNSET),
      mScale(DOUBLE_UNSET),
      mGeneration(0),
      mMaxGeneration(UNSIGNED_UNSET)
{
}

GammaDistributedGenerationalCellCycleModel::GammaDistributedGenerationalCellCycleModel(const GammaDistributedGenerationalCellCycleModel& rModel)
   :  AbstractSimpleCellCycleModel(rModel),
      mShape(rModel.mShape),
      mScale(rModel.mScale),
      mGeneration(rModel.mGeneration),
      mMaxGeneration(rModel.mMaxGeneration)
{
}

double GammaDistributedGenerationalCellCycleModel::GetAverageTransitCellCycleTime()
{
    return mShape*mScale;
}

double GammaDistributedGenerationalCellCycleModel::GetAverageStemCellCycleTime()
{
    return mShape*mScale;
}

AbstractCellCycleModel* GammaDistributedGenerationalCellCycleModel::CreateCellCycleModel()
{
    return new GammaDistributedGenerationalCellCycleModel(*this);
}

void GammaDistributedGenerationalCellCycleModel::ResetForDivision()
{
    mGeneration++;
    if (mGeneration > mMaxGeneration)
    {
        /*
         * This method is usually called within a CellBasedSimulation, after the CellPopulation
         * has called CellPropertyRegistry::TakeOwnership(). This means that were we to call
         * CellPropertyRegistry::Instance() here when setting the CellProliferativeType, we
         * would be creating a new CellPropertyRegistry. In this case the cell proliferative
         * type counts, as returned by AbstractCellPopulation::GetCellProliferativeTypeCount(),
         * would be incorrect. We must therefore access the CellProliferativeType via the cell's
         * CellPropertyCollection.
         */
        boost::shared_ptr<AbstractCellProperty> p_diff_type =
            mpCell->rGetCellPropertyCollection().GetCellPropertyRegistry()->Get<DifferentiatedCellProliferativeType>();
        mpCell->SetCellProliferativeType(p_diff_type);
    }
    AbstractSimpleCellCycleModel::ResetForDivision();
}

void GammaDistributedGenerationalCellCycleModel::InitialiseDaughterCell()
{
    if (mGeneration > mMaxGeneration)
    {
        boost::shared_ptr<AbstractCellProperty> p_diff_type =
            mpCell->rGetCellPropertyCollection().GetCellPropertyRegistry()->Get<DifferentiatedCellProliferativeType>();
        mpCell->SetCellProliferativeType(p_diff_type);
    }
    AbstractSimpleCellCycleModel::InitialiseDaughterCell();
}

void GammaDistributedGenerationalCellCycleModel::SetGeneration(unsigned generation)
{
    mGeneration = generation;
}

unsigned GammaDistributedGenerationalCellCycleModel::GetGeneration() const
{
    return mGeneration;
}

void GammaDistributedGenerationalCellCycleModel::SetMaxGeneration(unsigned maxGeneration)
{
    mMaxGeneration = maxGeneration;
}

unsigned GammaDistributedGenerationalCellCycleModel::GetMaxGeneration() const
{
    return mMaxGeneration;
}

void GammaDistributedGenerationalCellCycleModel::SetCellCycleDuration()
{
    if (    mpCell->GetCellProliferativeType()->IsType<StemCellProliferativeType>()
         || mpCell->GetCellProliferativeType()->IsType<TransitCellProliferativeType>() )
    {
        // Generate a gamma random number with mShape and mScale
        mCellCycleDuration = RandomNumberGenerator::Instance()->GammaRandomDeviate(mShape, mScale);
    }
    else if (mpCell->GetCellProliferativeType()->IsType<DifferentiatedCellProliferativeType>())
    {
        mCellCycleDuration = DBL_MAX;
    }
    else
    {
        NEVER_REACHED;
    }
}

void GammaDistributedGenerationalCellCycleModel::SetShape(double shape)
{
    mShape = shape;
}

void GammaDistributedGenerationalCellCycleModel::SetScale(double scale)
{
    mScale = scale;
}

double GammaDistributedGenerationalCellCycleModel::GetShape() const
{
    return mShape;
}

double GammaDistributedGenerationalCellCycleModel::GetScale() const
{
    return mScale;
}

void GammaDistributedGenerationalCellCycleModel::OutputCellCycleModelParameters(out_stream& rParamsFile)
{
    *rParamsFile << "\t\t\t<Shape>" << mShape << "</Shape>\n";
    *rParamsFile << "\t\t\t<Scale>" << mScale << "</Scale>\n";
    *rParamsFile << "\t\t\t<Generation>" << mGeneration << "</Generation>\n";
    *rParamsFile << "\t\t\t<MaxGeneration>" << mMaxGeneration << "</MaxGeneration>\n";

    AbstractSimpleCellCycleModel::OutputCellCycleModelParameters(rParamsFile);
}

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
CHASTE_CLASS_EXPORT(GammaDistributedGenerationalCellCycleModel)
