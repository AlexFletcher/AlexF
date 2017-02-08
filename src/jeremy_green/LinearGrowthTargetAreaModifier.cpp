
#include "LinearGrowthTargetAreaModifier.hpp"
#include "AbstractSimpleCellCycleModel.hpp"
#include "DifferentiatedCellProliferativeType.hpp"

template<unsigned DIM>
LinearGrowthTargetAreaModifier<DIM>::LinearGrowthTargetAreaModifier()
    : AbstractTargetAreaModifier<DIM>()
{
}

template<unsigned DIM>
LinearGrowthTargetAreaModifier<DIM>::~LinearGrowthTargetAreaModifier()
{
}

template<unsigned DIM>
void LinearGrowthTargetAreaModifier<DIM>::UpdateTargetAreaOfCell(CellPtr pCell)
{
    ///\todo assert that the cell is not differentiated and its cell-cycle model inherits from AbstractSimpleCellCycleModel

    double cell_age = pCell->GetAge();
    double cycle_time = (static_cast<AbstractSimpleCellCycleModel*>(pCell->GetCellCycleModel()))->GetCellCycleDuration();

    // The target area of a proliferating cell increases linearly from A to 2A over the cell cycle
    double cell_target_area = this->mReferenceTargetArea * (1.0 + cell_age/cycle_time);

    // Set cell data
    pCell->GetCellData()->SetItem("target area", cell_target_area);
}

template<unsigned DIM>
void LinearGrowthTargetAreaModifier<DIM>::OutputSimulationModifierParameters(out_stream& rParamsFile)
{
    // No parameters to output, so just call method on direct parent class
    AbstractTargetAreaModifier<DIM>::OutputSimulationModifierParameters(rParamsFile);
}

// Explicit instantiation
template class LinearGrowthTargetAreaModifier<1>;
template class LinearGrowthTargetAreaModifier<2>;
template class LinearGrowthTargetAreaModifier<3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(LinearGrowthTargetAreaModifier)
