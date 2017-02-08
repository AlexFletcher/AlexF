
#include "RandomGrowthTargetAreaModifier.hpp"
#include "AbstractSimpleCellCycleModel.hpp"
#include "RandomNumberGenerator.hpp"

template<unsigned DIM>
RandomGrowthTargetAreaModifier<DIM>::RandomGrowthTargetAreaModifier()
    : AbstractTargetAreaModifier<DIM>(),
      mMaxGrowthRate(0.0)
{
}

template<unsigned DIM>
RandomGrowthTargetAreaModifier<DIM>::~RandomGrowthTargetAreaModifier()
{
}

template<unsigned DIM>
void RandomGrowthTargetAreaModifier<DIM>::SetMaxGrowthRate(double maxGrowthRate)
{
    mMaxGrowthRate = maxGrowthRate;
}

template<unsigned DIM>
double RandomGrowthTargetAreaModifier<DIM>::GetMaxGrowthRate()
{
    return mMaxGrowthRate;
}

template<unsigned DIM>
void RandomGrowthTargetAreaModifier<DIM>::UpdateTargetAreaOfCell(CellPtr pCell)
{
    double growth_rate = RandomNumberGenerator::Instance()->ranf()*mMaxGrowthRate;
    double dt = SimulationTime::Instance()->GetTimeStep();
    double old_target_area = pCell->GetCellData()->GetItem("target area");
    double new_target_area = old_target_area + growth_rate*dt;

    pCell->GetCellData()->SetItem("target area", new_target_area);
}

template<unsigned DIM>
void RandomGrowthTargetAreaModifier<DIM>::OutputSimulationModifierParameters(out_stream& rParamsFile)
{
    *rParamsFile << "\t\t\t<MaxGrowthRate>" << mMaxGrowthRate << "</MaxGrowthRate>\n";
    AbstractTargetAreaModifier<DIM>::OutputSimulationModifierParameters(rParamsFile);
}

// Explicit instantiation
template class RandomGrowthTargetAreaModifier<1>;
template class RandomGrowthTargetAreaModifier<2>;
template class RandomGrowthTargetAreaModifier<3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(RandomGrowthTargetAreaModifier)
