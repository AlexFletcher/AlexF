
#include "TargetAreaModifierForAreaBasedCellCycleModel.hpp"
#include "AreaBasedCellCycleModel.hpp"

template<unsigned DIM>
TargetAreaModifierForAreaBasedCellCycleModel<DIM>::TargetAreaModifierForAreaBasedCellCycleModel()
    : AbstractTargetAreaModifier<DIM>()
{
}

template<unsigned DIM>
TargetAreaModifierForAreaBasedCellCycleModel<DIM>::~TargetAreaModifierForAreaBasedCellCycleModel()
{
}

template<unsigned DIM>
void TargetAreaModifierForAreaBasedCellCycleModel<DIM>::UpdateTargetAreaOfCell(CellPtr pCell)
{
	double cell_target_area = (static_cast<AreaBasedCellCycleModel*>(pCell->GetCellCycleModel()))->GetTargetArea();

    // Set cell data
    pCell->GetCellData()->SetItem("target area", cell_target_area);
}

template<unsigned DIM>
void TargetAreaModifierForAreaBasedCellCycleModel<DIM>::OutputSimulationModifierParameters(out_stream& rParamsFile)
{
    // No parameters to output, so just call method on direct parent class
    AbstractTargetAreaModifier<DIM>::OutputSimulationModifierParameters(rParamsFile);
}

// Explicit instantiation
template class TargetAreaModifierForAreaBasedCellCycleModel<1>;
template class TargetAreaModifierForAreaBasedCellCycleModel<2>;
template class TargetAreaModifierForAreaBasedCellCycleModel<3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(TargetAreaModifierForAreaBasedCellCycleModel)
