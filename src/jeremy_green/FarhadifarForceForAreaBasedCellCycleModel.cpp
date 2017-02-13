
#include "FarhadifarForceForAreaBasedCellCycleModel.hpp"
#include "AreaBasedCellCycleModel.hpp"

template<unsigned DIM>
FarhadifarForceForAreaBasedCellCycleModel<DIM>::FarhadifarForceForAreaBasedCellCycleModel()
   : FarhadifarForce<DIM>()
{
}

template<unsigned DIM>
FarhadifarForceForAreaBasedCellCycleModel<DIM>::~FarhadifarForceForAreaBasedCellCycleModel()
{
}

template<unsigned DIM>
void FarhadifarForceForAreaBasedCellCycleModel<DIM>::AddForceContribution(AbstractCellPopulation<DIM>& rCellPopulation)
{
	assert(dynamic_cast<VertexBasedCellPopulation<DIM>*>(&rCellPopulation) != NULL);

    // Define some helper variables
    VertexBasedCellPopulation<DIM>* p_cell_population = static_cast<VertexBasedCellPopulation<DIM>*>(&rCellPopulation);
    VertexMesh<DIM, DIM>& r_mesh = p_cell_population->rGetMesh();
    unsigned num_nodes = r_mesh.GetNumNodes();
    unsigned num_elements = r_mesh.GetNumElements();

    // Begin by computing the area and perimeter of each element in the mesh, to avoid having to do this multiple times
    std::vector<double> element_areas(num_elements);
    std::vector<double> element_perimeters(num_elements);
    std::vector<double> target_areas(num_elements);
    for (typename VertexMesh<DIM,DIM>::VertexElementIterator elem_iter = r_mesh.GetElementIteratorBegin();
         elem_iter != r_mesh.GetElementIteratorEnd();
         ++elem_iter)
    {
        unsigned elem_index = elem_iter->GetIndex();
        element_areas[elem_index] = r_mesh.GetVolumeOfElement(elem_index);
        element_perimeters[elem_index] = r_mesh.GetSurfaceAreaOfElement(elem_index);

        CellPtr p_cell = p_cell_population->GetCellUsingLocationIndex(elem_index);
        assert(dynamic_cast<AreaBasedCellCycleModel*>(p_cell->GetCellCycleModel()) != NULL);

        double target_area = static_cast<AreaBasedCellCycleModel*>(p_cell->GetCellCycleModel())->GetTargetArea();
        target_areas[elem_index] = target_area;
    }

    // Iterate over vertices in the cell population
    for (unsigned node_index=0; node_index<num_nodes; node_index++)
    {
        c_vector<double, DIM> area_elasticity_contribution = zero_vector<double>(DIM);
        c_vector<double, DIM> perimeter_contractility_contribution = zero_vector<double>(DIM);
        c_vector<double, DIM> line_tension_contribution = zero_vector<double>(DIM);

        // Find the indices of the elements owned by this node
        std::set<unsigned> containing_elem_indices = r_mesh.GetNode(node_index)->rGetContainingElementIndices();

        // Iterate over these elements
        for (std::set<unsigned>::iterator iter = containing_elem_indices.begin();
             iter != containing_elem_indices.end();
             ++iter)
        {
            // Get this element, its index and its number of nodes
            VertexElement<DIM, DIM>* p_element = r_mesh.GetElement(*iter);
            unsigned elem_index = p_element->GetIndex();

            // Find the local index of this node in this element
            unsigned local_index = p_element->GetNodeLocalIndex(node_index);

            // Add the force contribution from this cell's area elasticity (note the minus sign)
            c_vector<double, DIM> element_area_gradient = r_mesh.GetAreaGradientOfElementAtNode(p_element, local_index);
            area_elasticity_contribution -= this->GetAreaElasticityParameter()*(element_areas[elem_index] - target_areas[elem_index])*element_area_gradient;
        }

        r_mesh.GetNode(node_index)->AddAppliedForceContribution(area_elasticity_contribution);
    }

    // Iterate over all edges and add line tension and perimeter contractility force contributions
    for (typename VertexMesh<DIM, DIM>::EdgeIterator edge_iter = r_mesh.EdgesBegin();
         edge_iter != r_mesh.EdgesEnd();
         ++edge_iter)
    {
    	// Compute the line tension parameter for this edge (this is reset to the value for a non-boundary edge below, if required)
        double line_tension_parameter = this->GetBoundaryLineTensionParameter();

        unsigned node_A_index = edge_iter.GetNodeA()->GetIndex();
        unsigned node_B_index = edge_iter.GetNodeB()->GetIndex();

        const c_vector<double, DIM>& r_node_A_location = edge_iter.GetNodeA()->rGetLocation();
        const c_vector<double, DIM>& r_node_B_location = edge_iter.GetNodeB()->rGetLocation();

        c_vector<double, DIM> edge_vector = r_mesh.GetVectorFromAtoB(r_node_B_location, r_node_A_location);
        double edge_length = norm_2(edge_vector);

        assert(edge_length > DBL_EPSILON);

        c_vector<double, DIM> edge_gradient = edge_vector/edge_length;

        // Perimeter contractility term
        unsigned elem_index = edge_iter.GetElemIndex();
        c_vector<double, DIM> perimeter_contractility_force_on_node_A = -this->GetPerimeterContractilityParameter()* element_perimeters[elem_index]*edge_gradient;
        c_vector<double, DIM> perimeter_contractility_force_on_node_B = -perimeter_contractility_force_on_node_A;
        r_mesh.GetNode(node_A_index)->AddAppliedForceContribution(perimeter_contractility_force_on_node_A);
        r_mesh.GetNode(node_B_index)->AddAppliedForceContribution(perimeter_contractility_force_on_node_B);

        unsigned other_elem_index = edge_iter.GetOtherElemIndex();
        if (other_elem_index != UINT_MAX)
        {
        	line_tension_parameter = this->GetLineTensionParameter();

            perimeter_contractility_force_on_node_A = -this->GetPerimeterContractilityParameter()* element_perimeters[other_elem_index]*edge_gradient;
            perimeter_contractility_force_on_node_B = -perimeter_contractility_force_on_node_A;
            r_mesh.GetNode(node_A_index)->AddAppliedForceContribution(perimeter_contractility_force_on_node_A);
            r_mesh.GetNode(node_B_index)->AddAppliedForceContribution(perimeter_contractility_force_on_node_B);
        }

        // Line tension term
        c_vector<double, DIM> line_tension_force_on_node_A = -line_tension_parameter*edge_gradient;
        c_vector<double, DIM> line_tension_force_on_node_B = -line_tension_force_on_node_A;
        r_mesh.GetNode(node_A_index)->AddAppliedForceContribution(line_tension_force_on_node_A);
        r_mesh.GetNode(node_B_index)->AddAppliedForceContribution(line_tension_force_on_node_B);
    }
}

template<unsigned DIM>
void FarhadifarForceForAreaBasedCellCycleModel<DIM>::OutputForceParameters(out_stream& rParamsFile)
{
    FarhadifarForce<DIM>::OutputForceParameters(rParamsFile);
}

// Explicit instantiation
template class FarhadifarForceForAreaBasedCellCycleModel<1>;
template class FarhadifarForceForAreaBasedCellCycleModel<2>;
template class FarhadifarForceForAreaBasedCellCycleModel<3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(FarhadifarForceForAreaBasedCellCycleModel)
