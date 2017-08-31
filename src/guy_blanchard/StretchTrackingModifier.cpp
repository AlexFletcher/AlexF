
#include "StretchTrackingModifier.hpp"
#include "VertexBasedCellPopulation.hpp"

template<unsigned DIM>
StretchTrackingModifier<DIM>::StretchTrackingModifier()
    : AbstractCellBasedSimulationModifier<DIM>()
{
}

template<unsigned DIM>
StretchTrackingModifier<DIM>::~StretchTrackingModifier()
{
}

template<unsigned DIM>
void StretchTrackingModifier<DIM>::UpdateAtEndOfTimeStep(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
{
    UpdateCellData(rCellPopulation);
}

template<unsigned DIM>
void StretchTrackingModifier<DIM>::SetupSolve(AbstractCellPopulation<DIM,DIM>& rCellPopulation, std::string outputDirectory)
{
    UpdateCellData(rCellPopulation);
}

template<unsigned DIM>
void StretchTrackingModifier<DIM>::UpdateCellData(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
{
	VertexBasedCellPopulation<DIM>* p_cell_population = static_cast<VertexBasedCellPopulation<DIM>*>(&rCellPopulation);

    // Iterate over cell population
    for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = p_cell_population->Begin();
         cell_iter != p_cell_population->End();
         ++cell_iter)
    {
    	double stretch = 0.0;

        // Get the element corresponding to this cell
    	VertexElement<DIM, DIM>* p_element = p_cell_population->GetElementCorrespondingToCell(*cell_iter);

    	for (unsigned i=0; i<p_element->GetNumNodes()-1; i++)
    	{
    		c_vector<double, DIM> location_i = p_element->GetNode(i)->rGetLocation();
    		for (unsigned j=i+1; j<p_element->GetNumNodes(); j++)
    		{
    			c_vector<double, DIM> location_j = p_element->GetNode(j)->rGetLocation();
    			double distance = norm_2(p_cell_population->rGetMesh().GetVectorFromAtoB(location_i,location_j));
    			if (distance > stretch)
    			{
    				stretch = distance;
    			}
    		}
    	}

        // Store the cell's volume in CellData
        cell_iter->GetCellData()->SetItem("stretch", stretch);
    }
}

template<unsigned DIM>
void StretchTrackingModifier<DIM>::OutputSimulationModifierParameters(out_stream& rParamsFile)
{
    // No parameters to output, so just call method on direct parent class
    AbstractCellBasedSimulationModifier<DIM>::OutputSimulationModifierParameters(rParamsFile);
}

// Explicit instantiation
template class StretchTrackingModifier<1>;
template class StretchTrackingModifier<2>;
template class StretchTrackingModifier<3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(StretchTrackingModifier)

