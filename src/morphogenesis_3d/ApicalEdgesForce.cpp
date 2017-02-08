
#include "ApicalEdgesForce.hpp"
#include "Debug.hpp"

template<unsigned DIM>
ApicalEdgesForce<DIM>::ApicalEdgesForce()
    : AbstractForce<DIM>(),
      mLambda(1.0)
{
}

template<unsigned DIM>
ApicalEdgesForce<DIM>::~ApicalEdgesForce()
{
}

template<unsigned DIM>
void ApicalEdgesForce<DIM>::SetLambda(double lambda)
{
    mLambda = lambda;
}

template<unsigned DIM>
double ApicalEdgesForce<DIM>::GetLambda()
{
    return mLambda;
}

template<unsigned DIM>
void ApicalEdgesForce<DIM>::AddForceContribution(AbstractCellPopulation<DIM>& rCellPopulation)
{
    // Throw an exception message if not using a VertexBasedCellPopulation
    if (dynamic_cast<VertexBasedCellPopulation<DIM>*>(&rCellPopulation) == NULL)
    {
        EXCEPTION("ApicalEdgesForce is to be used with a VertexBasedCellPopulation only");
    }

    // Define some helper variables
    VertexBasedCellPopulation<DIM>* p_cell_population = static_cast<VertexBasedCellPopulation<DIM>*>(&rCellPopulation);
    MutableVertexMesh<DIM, DIM>& r_mesh = p_cell_population->rGetMesh();
    unsigned num_nodes = p_cell_population->GetNumNodes();

    // Iterate over vertices in the cell population
    ///\todo see starting value in for loop below
    for (unsigned node_index=num_nodes/2; node_index<num_nodes; node_index++)
    {
        /*
         * The apical line tension energy is associated with apical Nodes. Here we define apical
         * nodes are those with NodeAttributes, as specified by the HexagonalPrism3dVertexMeshGenerator
         * used to construct the mesh. (Note that if there are no rearrangements or re-indexing
         * then we could just use the fact that the first half of the nodes vector are apical.)
         */
        Node<DIM>* p_this_node = p_cell_population->GetNode(node_index);
        if (p_this_node->HasNodeAttributes())
        {
            /*
             * The force on this Node is given by the gradient of the total apical line tension
             * energy of the CellPopulation, evaluated at the Node's position. Since the movement
             * of this Node only affects the free energy associated with the VertexElements that
             * contain it, we can just consider the contributions to the free energy gradient from
             * each of these VertexElements.
             */
            c_vector<double, DIM> force_on_node = zero_vector<double>(DIM);

            // Find the indices of the elements containing this node
            std::set<unsigned> containing_elem_indices = p_this_node->rGetContainingElementIndices();

            // If the node is contained in less than three elements, then 'pin' it by imposing no force
            if (containing_elem_indices.size() > 2)
            {
                // Iterate over these elements
                for (std::set<unsigned>::iterator iter = containing_elem_indices.begin();
                     iter != containing_elem_indices.end();
                     ++iter)
                {
                    // Get this element, its index and its number of nodes
                    VertexElement<DIM, DIM>* p_element = p_cell_population->GetElement(*iter);
//                    unsigned elem_index = p_element->GetIndex();
//                    unsigned num_nodes_elem = p_element->GetNumNodes();

                    // Find the local index of this node in this element
                    unsigned local_index = p_element->GetNodeLocalIndex(node_index);

                    // Get the previous and next nodes in this element
                    unsigned previous_node_local_index = local_index - 1;
                    unsigned next_node_local_index = local_index + 1;
                    if (local_index == 6)
                    {
                        previous_node_local_index = 11;
                    }
                    else if (local_index == 11)
                    {
                        next_node_local_index = 6;
                    }

                    Node<DIM>* p_previous_node = p_element->GetNode(previous_node_local_index);
                    Node<DIM>* p_next_node = p_element->GetNode(next_node_local_index);

                    /**
                     * Compute the line tension parameter for each of these edges
                     * \todo be aware that this is half of the actual value for internal edges,
                     * since we are looping over each of the internal edges twice (see the
                     * method FarhadifarForce::GetLineTensionParameter())
                     */
                    double line_tension = mLambda;

                    // Compute the gradient of each these edges, computed at the present node
                    c_vector<double, DIM> previous_edge_gradient = r_mesh.GetVectorFromAtoB(p_previous_node->rGetLocation(), p_this_node->rGetLocation());
                    double previous_edge_length = norm_2(previous_edge_gradient);
                    assert(previous_edge_length > DBL_EPSILON);
                    previous_edge_gradient /= previous_edge_length;

                    c_vector<double, DIM> next_edge_gradient = r_mesh.GetVectorFromAtoB(p_this_node->rGetLocation(), p_next_node->rGetLocation());
                    double next_edge_length = norm_2(next_edge_gradient);
                    assert(next_edge_length > DBL_EPSILON);
                    next_edge_gradient /= next_edge_length;

                    // Add the force contribution from cell-cell and cell-boundary line tension (note the minus sign)
                    force_on_node += -line_tension*previous_edge_gradient + line_tension*next_edge_gradient;
                }

                p_cell_population->GetNode(node_index)->AddAppliedForceContribution(force_on_node);
            }
        }
    }
}

template<unsigned DIM>
void ApicalEdgesForce<DIM>::OutputForceParameters(out_stream& rParamsFile)
{
    *rParamsFile << "\t\t\t<Lambda>" << mLambda << "</Lambda>\n";

    // Call method on direct parent class
    AbstractForce<DIM>::OutputForceParameters(rParamsFile);
}

// Explicit instantiation
template class ApicalEdgesForce<1>;
template class ApicalEdgesForce<2>;
template class ApicalEdgesForce<3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(ApicalEdgesForce)
