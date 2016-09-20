/*

Copyright (c) 2005-2016, University of Oxford.
All rights reserved.

University of Oxford means the Chancellor, Masters and Scholars of the
University of Oxford, having an administrative office at Wellington
Square, Oxford OX1 2JD, UK.

This file is part of Chaste.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of the University of Oxford nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "NagaiHondaMultipleLabelsForce.hpp"
#include "CellLabel.hpp"

template<unsigned DIM>
NagaiHondaMultipleLabelsForce<DIM>::NagaiHondaMultipleLabelsForce()
    : NagaiHondaForce<DIM>(),
      mUseExponentialLineTension(false),
      mCellBoundaryAdhesionParameter(1.0),
      mHomotypicCellAdhesionParameter(1.0),
      mHeterotypicCellAdhesionParameter(1.0),
      mLambdaParameter(1.0),
      mNumLabelledColours(UNSIGNED_UNSET)
{
}

template<unsigned DIM>
NagaiHondaMultipleLabelsForce<DIM>::~NagaiHondaMultipleLabelsForce()
{
}

template<unsigned DIM>
bool NagaiHondaMultipleLabelsForce<DIM>::GetUseExponentialLineTension()
{
    return mUseExponentialLineTension;
}

template<unsigned DIM>
void NagaiHondaMultipleLabelsForce<DIM>::SetUseExponentialLineTension(bool useExponentialLineTension)
{
    mUseExponentialLineTension = useExponentialLineTension;
}

template<unsigned DIM>
void NagaiHondaMultipleLabelsForce<DIM>::AddForceContribution(AbstractCellPopulation<DIM>& rCellPopulation)
{
    // Throw an exception message if not using a VertexBasedCellPopulation
    if (dynamic_cast<VertexBasedCellPopulation<DIM>*>(&rCellPopulation) == NULL)
    {
        EXCEPTION("NagaiHondaMultipleLabelsForce is to be used with a VertexBasedCellPopulation only");
    }

    // Define some helper variables
    VertexBasedCellPopulation<DIM>* p_cell_population = static_cast<VertexBasedCellPopulation<DIM>*>(&rCellPopulation);
    unsigned num_nodes = p_cell_population->GetNumNodes();
    unsigned num_elements = p_cell_population->GetNumElements();

    // Begin by computing the area and perimeter of each element in the mesh, to avoid having to do this multiple times
    std::vector<double> element_areas(num_elements);
    std::vector<double> element_perimeters(num_elements);
    std::vector<double> target_areas(num_elements);
    for (typename VertexMesh<DIM,DIM>::VertexElementIterator elem_iter = p_cell_population->rGetMesh().GetElementIteratorBegin();
         elem_iter != p_cell_population->rGetMesh().GetElementIteratorEnd();
         ++elem_iter)
    {
        unsigned elem_index = elem_iter->GetIndex();
        element_areas[elem_index] = p_cell_population->rGetMesh().GetVolumeOfElement(elem_index);
        element_perimeters[elem_index] = p_cell_population->rGetMesh().GetSurfaceAreaOfElement(elem_index);
        try
        {
            target_areas[elem_index] = p_cell_population->GetCellUsingLocationIndex(elem_index)->GetCellData()->GetItem("target area");
        }
        catch (Exception&)
        {
            EXCEPTION("You need to add an AbstractTargetAreaModifier to the simulation in order to use NagaiHondaMultipleLabelsForce");
        }
    }

    // Compute the force contribution on each vertex arising from the cell deformation energy
    // (target area) and membrane surface tension energy (perimeter contractility)
    for (unsigned node_index=0; node_index<num_nodes; node_index++)
    {
        /*
         * The force on this Node is given by the gradient of the total free
         * energy of the CellPopulation, evaluated at the position of the vertex. This
         * free energy is the sum of the free energies of all CellPtrs in
         * the cell population. The free energy of each CellPtr is comprised of three
         * parts - a cell deformation energy, a membrane surface tension energy
         * and an adhesion energy.
         *
         * Note that since the movement of this Node only affects the free energy
         * of the Cells containing it, we can just consider the contributions
         * to the free energy gradient from each of these Cells.
         */
        c_vector<double, DIM> deformation_contribution = zero_vector<double>(DIM);
        c_vector<double, DIM> membrane_surface_tension_contribution = zero_vector<double>(DIM);

        // Find the indices of the elements owned by this node
        std::set<unsigned> containing_elem_indices = p_cell_population->GetNode(node_index)->rGetContainingElementIndices();

        // Iterate over these elements
        for (std::set<unsigned>::iterator iter = containing_elem_indices.begin();
             iter != containing_elem_indices.end();
             ++iter)
        {
            // Get this element, its index and its number of nodes
            VertexElement<DIM, DIM>* p_element = p_cell_population->GetElement(*iter);
            unsigned elem_index = p_element->GetIndex();
            unsigned num_nodes_elem = p_element->GetNumNodes();

            // Find the local index of this node in this element
            unsigned local_index = p_element->GetNodeLocalIndex(node_index);

            // Add the force contribution from this cell's deformation energy (note the minus sign)
            c_vector<double, DIM> element_area_gradient = p_cell_population->rGetMesh().GetAreaGradientOfElementAtNode(p_element, local_index);
            deformation_contribution -= 2*this->GetNagaiHondaDeformationEnergyParameter()*(element_areas[elem_index] - target_areas[elem_index])*element_area_gradient;

            // Get the previous and next nodes in this element
            unsigned previous_node_local_index = (num_nodes_elem+local_index-1)%num_nodes_elem;

            // Compute the gradient of each these edges, computed at the present node
            c_vector<double, DIM> previous_edge_gradient = -p_cell_population->rGetMesh().GetNextEdgeGradientOfElementAtNode(p_element, previous_node_local_index);
            c_vector<double, DIM> next_edge_gradient = p_cell_population->rGetMesh().GetNextEdgeGradientOfElementAtNode(p_element, local_index);

            // Add the force contribution from this cell's membrane surface tension (note the minus sign)
            c_vector<double, DIM> element_perimeter_gradient = previous_edge_gradient + next_edge_gradient;
            double cell_target_perimeter = 2*sqrt(M_PI*target_areas[elem_index]);
            membrane_surface_tension_contribution -= 2*this->GetNagaiHondaMembraneSurfaceEnergyParameter()*(element_perimeters[elem_index] - cell_target_perimeter)*element_perimeter_gradient;
        }

        c_vector<double, DIM> force_on_node = deformation_contribution + membrane_surface_tension_contribution;
        p_cell_population->GetNode(node_index)->AddAppliedForceContribution(force_on_node);
    }

    // Compute the force contribution on each vertex arising from the adhesion energy (line tension)
    for (unsigned node_index=0; node_index<num_nodes; node_index++)
    {
        Node<DIM>* p_this_node = p_cell_population->GetNode(node_index);

        c_vector<double, DIM> adhesion_contribution = zero_vector<double>(DIM);

        // Find the indices of the elements owned by this node
        std::set<unsigned> containing_elem_indices = p_cell_population->GetNode(node_index)->rGetContainingElementIndices();

        // Iterate over these elements
        for (std::set<unsigned>::iterator iter = containing_elem_indices.begin();
             iter != containing_elem_indices.end();
             ++iter)
        {
            // Get this element, its index and its number of nodes
            VertexElement<DIM, DIM>* p_element = p_cell_population->GetElement(*iter);
            unsigned num_nodes_elem = p_element->GetNumNodes();

            // Find the local index of this node in this element
            unsigned local_index = p_element->GetNodeLocalIndex(node_index);

            // Get the previous and next nodes in this element
            unsigned previous_node_local_index = (num_nodes_elem+local_index-1)%num_nodes_elem;
            Node<DIM>* p_previous_node = p_element->GetNode(previous_node_local_index);

            unsigned next_node_local_index = (local_index+1)%num_nodes_elem;
            Node<DIM>* p_next_node = p_element->GetNode(next_node_local_index);

            // Compute the adhesion parameter for each of these edges
            double previous_edge_adhesion_parameter = GetAdhesionParameter(p_previous_node, p_this_node, *p_cell_population);
            double next_edge_adhesion_parameter = GetAdhesionParameter(p_this_node, p_next_node, *p_cell_population);

            // Compute the gradient of each these edges, computed at the present node
            c_vector<double, DIM> previous_edge_gradient = -p_cell_population->rGetMesh().GetNextEdgeGradientOfElementAtNode(p_element, previous_node_local_index);
            c_vector<double, DIM> next_edge_gradient = p_cell_population->rGetMesh().GetNextEdgeGradientOfElementAtNode(p_element, local_index);

            /*
             * Suppose we have an adhesion energy term U_A that is the sum over edges (i,j) of
             * contributions
             *
             * gamma_ij * (1 - exp(-lambda * l_ij)),
             *
             * where l_ij is the length of edge (i,j) and gamma_ij and lambda are parameters.
             * In this case, the force associated with U_A on vertex k is given by the sum over
             * edges (k,j) containing vertex k of contributions
             *
             * lambda * gamma_kj * exp(-lambda * l_ij) * grad(l_kj),
             *
             * where grad(l_kj) = (r_k - r_j)/l_kj is the gradient of l_ij and r_k, r_j denote
             * the position vectors of vertices k and j, respectively.
             */

            // Add the force contribution from cell-cell and cell-boundary adhesion (note the minus sign)
            if (mUseExponentialLineTension)
            {
                double previous_edge_length = p_cell_population->rGetMesh().GetDistanceBetweenNodes(p_previous_node->GetIndex(), p_this_node->GetIndex());
                double next_edge_length = p_cell_population->rGetMesh().GetDistanceBetweenNodes(p_this_node->GetIndex(), p_next_node->GetIndex());
                adhesion_contribution -=   previous_edge_adhesion_parameter*mLambdaParameter*exp(-mLambdaParameter*previous_edge_length)*previous_edge_gradient
                                         + next_edge_adhesion_parameter*mLambdaParameter*exp(-mLambdaParameter*next_edge_length)*next_edge_gradient;
            }
            else
            {
                adhesion_contribution -= previous_edge_adhesion_parameter*previous_edge_gradient + next_edge_adhesion_parameter*next_edge_gradient;
            }
        }

        c_vector<double, DIM> force_on_node = adhesion_contribution;
        p_cell_population->GetNode(node_index)->AddAppliedForceContribution(force_on_node);
    }
}

template<unsigned DIM>
unsigned NagaiHondaMultipleLabelsForce<DIM>::GetDifferenceInLabelsAcrossEdge(Node<DIM>* pNodeA,
                                                                             Node<DIM>* pNodeB,
                                                                             VertexBasedCellPopulation<DIM>& rVertexCellPopulation)
{
    ///\todo For simplicity, we assume that every cell in the population has SOME cell label

    // If this is the first time we have called GetAdhesionParameter() in the simulation...
    if (mNumLabelledColours == UNSIGNED_UNSET)
    {
        // ...then compute the number of different cell label colours present, and store this in mNumLabelledColours
        ComputeNumLabelledColours(rVertexCellPopulation);
    }

    // Find the indices of the elements owned by each node
    std::set<unsigned> elements_containing_nodeA = pNodeA->rGetContainingElementIndices();
    std::set<unsigned> elements_containing_nodeB = pNodeB->rGetContainingElementIndices();

    // Find common elements
    std::set<unsigned> shared_elements;
    std::set_intersection(elements_containing_nodeA.begin(), elements_containing_nodeA.end(),
                          elements_containing_nodeB.begin(), elements_containing_nodeB.end(),
                          std::inserter(shared_elements, shared_elements.begin()));

    // Check that the nodes have a common edge
    assert(!shared_elements.empty());

    // If the edge corresponds to a single element, then the cell is on the boundary, so return UNSIGNED_UNSET
    if (shared_elements.size() == 1)
    {
        return UNSIGNED_UNSET;
    }
    else
    {
        // Get the colour of the label associated with cell 1
        unsigned colour_of_cell_1 = 0;
        std::set<unsigned>::iterator iter = shared_elements.begin();
        unsigned element_index = *(iter);
        CellPtr p_cell = rVertexCellPopulation.GetCellUsingLocationIndex(element_index);

        assert(p_cell->template HasCellProperty<CellLabel>());
        CellPropertyCollection collection = p_cell->rGetCellPropertyCollection().GetProperties<CellLabel>();
        boost::shared_ptr<CellLabel> p_label = boost::static_pointer_cast<CellLabel>(collection.GetProperty());
        colour_of_cell_1 = p_label->GetColour();

        // Get the colour of the label associated with cell 2
        ++iter;
        unsigned colour_of_cell_2 = 0;
        element_index = *(iter);
        CellPtr p_cell2 = rVertexCellPopulation.GetCellUsingLocationIndex(element_index);

        assert(p_cell->template HasCellProperty<CellLabel>());
        CellPropertyCollection collection2 = p_cell2->rGetCellPropertyCollection().GetProperties<CellLabel>();
        boost::shared_ptr<CellLabel> p_label2 = boost::static_pointer_cast<CellLabel>(collection2.GetProperty());
        colour_of_cell_2 = p_label2->GetColour();

        unsigned difference_in_labels = abs(colour_of_cell_1%mNumLabelledColours - colour_of_cell_2%mNumLabelledColours);
        return difference_in_labels;
    }
}

template<unsigned DIM>
double NagaiHondaMultipleLabelsForce<DIM>::GetAdhesionParameter(Node<DIM>* pNodeA,
                                                                Node<DIM>* pNodeB,
                                                                VertexBasedCellPopulation<DIM>& rVertexCellPopulation)
{
    unsigned difference_in_labels = GetDifferenceInLabelsAcrossEdge(pNodeA, pNodeB, rVertexCellPopulation);

    if (difference_in_labels == UNSIGNED_UNSET)
    {
        // In this case, this edge belongs to a single element so is on the boundary
        return this->GetCellBoundaryAdhesionParameter();
    }
    else if ((pNodeA->IsBoundaryNode()) || (pNodeB->IsBoundaryNode()))
    {
        return this->GetCellBoundaryAdhesionParameter();
    }
    else if (difference_in_labels == 0)
    {
        // In this case, the cells sharing this edge have the same label colour
        return this->GetHomotypicCellAdhesionParameter();
    }
    else
    {
        // In this case, the cells sharing this edge have different label colours, so we return
        // mHeterotypicCellAdhesionParameter scaled by a multiplier defined by the difference
        // in label colours
        double energy_parameter_multiplier = (double)(difference_in_labels); // 26 SEP 2015 // 1.0;

        // Label numbers wrap around, so check to find smallest difference
        if (energy_parameter_multiplier > mNumLabelledColours/2)
        {
            energy_parameter_multiplier = mNumLabelledColours - energy_parameter_multiplier;
        }

        return this->GetHeterotypicCellAdhesionParameter() * energy_parameter_multiplier;
    }
}

template<unsigned DIM>
double NagaiHondaMultipleLabelsForce<DIM>::GetCellBoundaryAdhesionParameter()
{
    return mCellBoundaryAdhesionParameter;
}

template<unsigned DIM>
double NagaiHondaMultipleLabelsForce<DIM>::GetHomotypicCellAdhesionParameter()
{
    return mHomotypicCellAdhesionParameter;
}

template<unsigned DIM>
double NagaiHondaMultipleLabelsForce<DIM>::GetHeterotypicCellAdhesionParameter()
{
    return mHeterotypicCellAdhesionParameter;
}

template<unsigned DIM>
double NagaiHondaMultipleLabelsForce<DIM>::GetLambdaParameter()
{
    return mLambdaParameter;
}

template<unsigned DIM>
unsigned NagaiHondaMultipleLabelsForce<DIM>::GetNumLabelledColours()
{
    return mNumLabelledColours;
}

template<unsigned DIM>
void NagaiHondaMultipleLabelsForce<DIM>::SetCellBoundaryAdhesionParameter(double adhesionEnergyParameterForCellOnBoundary)
{
    mCellBoundaryAdhesionParameter = adhesionEnergyParameterForCellOnBoundary;
}

template<unsigned DIM>
void NagaiHondaMultipleLabelsForce<DIM>::SetHomotypicCellAdhesionParameter(double labelledCellLabelledCellAdhesionEnergyParameter)
{
    mHomotypicCellAdhesionParameter = labelledCellLabelledCellAdhesionEnergyParameter;
}

template<unsigned DIM>
void NagaiHondaMultipleLabelsForce<DIM>::SetHeterotypicCellAdhesionParameter(double labelledCellCellAdhesionEnergyParameter)
{
    mHeterotypicCellAdhesionParameter = labelledCellCellAdhesionEnergyParameter;
}

template<unsigned DIM>
void NagaiHondaMultipleLabelsForce<DIM>::SetLambdaParameter(double lambdaParameter)
{
    mLambdaParameter = lambdaParameter;
}

//\guy 02/02
template<unsigned DIM>
void NagaiHondaMultipleLabelsForce<DIM>::ComputeNumLabelledColours(VertexBasedCellPopulation<DIM>& rVertexCellPopulation)
{
    // Create a set of cell label colours that are present in the cell population
    std::set<unsigned> cell_label_colours_present;

    // Iterate over cells in the population...
    for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = rVertexCellPopulation.Begin();
         cell_iter != rVertexCellPopulation.End();
         ++cell_iter)
    {
        // ...and add its cell label colour to the set, if the cell is labelled
        if (cell_iter->template HasCellProperty<CellLabel>())
        {
            CellPropertyCollection collection = cell_iter->rGetCellPropertyCollection().template GetProperties<CellLabel>();
            boost::shared_ptr<CellLabel> p_label = boost::static_pointer_cast<CellLabel>(collection.GetProperty());
            cell_label_colours_present.insert(p_label->GetColour());
        }
    }

    // The number of distinct cell label colours is the size of the set
    mNumLabelledColours = cell_label_colours_present.size();
}

template<unsigned DIM>
void NagaiHondaMultipleLabelsForce<DIM>::OutputForceParameters(out_stream& rParamsFile)
{
    // Output member variables
    *rParamsFile << "\t\t\t<UseExponentialLineTension>" << mUseExponentialLineTension << "</UseExponentialLineTension> \n";
    *rParamsFile << "\t\t\t<CellBoundaryAdhesionParameter>" << mCellBoundaryAdhesionParameter << "</CellBoundaryAdhesionParameter> \n";
    *rParamsFile << "\t\t\t<HomotypicCellAdhesionParameter>" << mHomotypicCellAdhesionParameter << "</HomotypicCellAdhesionParameter> \n";
    *rParamsFile << "\t\t\t<HeterotypicCellAdhesionParameter>" << mHeterotypicCellAdhesionParameter << "</HeterotypicCellAdhesionParameter> \n";
    *rParamsFile << "\t\t\t<LambdaParameter>" << mLambdaParameter << "</LambdaParameter> \n";
    *rParamsFile << "\t\t\t<NumLabelledColours>" << mNumLabelledColours << "</NumLabelledColours> \n";

    NagaiHondaForce<DIM>::OutputForceParameters(rParamsFile);
}

// Explicit instantiation
template class NagaiHondaMultipleLabelsForce<1>;
template class NagaiHondaMultipleLabelsForce<2>;
template class NagaiHondaMultipleLabelsForce<3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(NagaiHondaMultipleLabelsForce)
