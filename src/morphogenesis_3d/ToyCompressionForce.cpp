
#include "ToyCompressionForce.hpp"
#include "Debug.hpp"

template<unsigned DIM>
ToyCompressionForce<DIM>::ToyCompressionForce()
    : AbstractForce<DIM>()
{
}

template<unsigned DIM>
ToyCompressionForce<DIM>::~ToyCompressionForce()
{
}

template<unsigned DIM>
void ToyCompressionForce<DIM>::AddForceContribution(AbstractCellPopulation<DIM>& rCellPopulation)
{
    // Iterate over vertices in the cell population
    unsigned num_nodes = rCellPopulation.GetNumNodes();
    for (unsigned node_index=0; node_index<num_nodes; node_index++)
    {
        c_vector<double, DIM> force_on_node = zero_vector<double>(DIM);
        force_on_node[DIM-1] = -0.5*rCellPopulation.GetNode(node_index)->rGetLocation()[1];

        rCellPopulation.GetNode(node_index)->AddAppliedForceContribution(force_on_node);
    }
}

template<unsigned DIM>
void ToyCompressionForce<DIM>::OutputForceParameters(out_stream& rParamsFile)
{
    // Call method on direct parent class
    AbstractForce<DIM>::OutputForceParameters(rParamsFile);
}

// Explicit instantiation
template class ToyCompressionForce<1>;
template class ToyCompressionForce<2>;
template class ToyCompressionForce<3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(ToyCompressionForce)
