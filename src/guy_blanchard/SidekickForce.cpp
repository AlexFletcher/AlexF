/*

Copyright (c) 2005-2017, University of Oxford.
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

#include "SidekickForce.hpp"

template<unsigned DIM>
SidekickForce<DIM>::SidekickForce()
    : FarhadifarForce<DIM>(),
      mHomotypicLineTensionParameter(1.0),
      mHeterotypicLineTensionParameter(1.0),
      mSupercontractileLineTensionParameter(1.0),
      mNumStripes(4)
{
}

template<unsigned DIM>
double SidekickForce<DIM>::GetLineTensionParameter(Node<DIM>* pNodeA,
                                                    Node<DIM>* pNodeB,
                                                    VertexBasedCellPopulation<DIM>& rVertexCellPopulation)
{
    double line_tension = 0.0;

    // If either node is a boundary node, then use mBoundaryLineTensionParameter...
    if (pNodeA->IsBoundaryNode() || pNodeB->IsBoundaryNode())
    {
        line_tension = this->mBoundaryLineTensionParameter;
    }
    else
    {
        // ...otherwise find the elements shared by these nodes
        std::set<unsigned> elements_containing_nodeA = pNodeA->rGetContainingElementIndices();
        std::set<unsigned> elements_containing_nodeB = pNodeB->rGetContainingElementIndices();
        std::set<unsigned> shared_elements;
        std::set_intersection(elements_containing_nodeA.begin(), elements_containing_nodeA.end(),
                              elements_containing_nodeB.begin(), elements_containing_nodeB.end(),
                              std::inserter(shared_elements, shared_elements.begin()));
        assert(!shared_elements.empty());

        // If the nodes share a single edge, then it must be on the boundary, so use mBoundaryLineTensionParameter...
        if (shared_elements.size() == 1)
        {
            line_tension = this->mBoundaryLineTensionParameter;
        }
        else
        {
            // ...otherwise proceed
            assert(shared_elements.size() == 2);
            unsigned elem_1_index = *(shared_elements.begin());
            unsigned elem_2_index = *(++(shared_elements.begin()));

            // Find the stripe identities on either side of the edge
            CellPtr p_cell_1 = rVertexCellPopulation.GetCellUsingLocationIndex(elem_1_index);
            CellPtr p_cell_2 = rVertexCellPopulation.GetCellUsingLocationIndex(elem_2_index);
            unsigned cell_1_stripe_identity = p_cell_1->GetCellData()->GetItem("stripe");
            unsigned cell_2_stripe_identity = p_cell_2->GetCellData()->GetItem("stripe");

            // If the edge is within a stripe (an 'internal' interface), then use mHomotypicLineTensionParameter
            if (cell_1_stripe_identity == cell_2_stripe_identity)
            {
                // The factor of 0.5 arises because each internal interface is visited twice in our iteration
                line_tension = 0.5*mHomotypicLineTensionParameter;
            }
            else
            {
                // Label numbers wrap around, so check to find smallest difference in stripe identities
                unsigned mismatch = abs(cell_1_stripe_identity - cell_2_stripe_identity);
                if (mismatch > mNumStripes/2)
                {
                    mismatch = mNumStripes - mismatch;
                }

                // If the edge is between stripes whose identities differ by 1, then use mHeterotypicLineTensionParameter
                if (mismatch == 1)
                {
                    // The factor of 0.5 arises because each internal interface is visited twice in our iteration
                    line_tension = 0.5*mHeterotypicLineTensionParameter;
                }
                else
                {
                    /* Regardless of how many stripes there actually are, the edge is between stripes whose identities
                     * differ by more than 1, then use mSupercontractileLineTensionParameter.
                     * Note: To 'turn off' super-contractility, we simply set mSupercontractileLineTensionParameter to
                     * equal mHeterotypicLineTensionParameter.
                     */

                    // The factor of 0.5 arises because each internal interface is visited twice in our iteration
                    line_tension = 0.5*mSupercontractileLineTensionParameter;
                }
            }
        }
    }

    return line_tension;
}

template<unsigned DIM>
void SidekickForce<DIM>::SetHomotypicLineTensionParameter(double labelledCellLabelledCellAdhesionEnergyParameter)
{
    mHomotypicLineTensionParameter = labelledCellLabelledCellAdhesionEnergyParameter;
}

template<unsigned DIM>
void SidekickForce<DIM>::SetHeterotypicLineTensionParameter(double labelledCellCellAdhesionEnergyParameter)
{
    mHeterotypicLineTensionParameter = labelledCellCellAdhesionEnergyParameter;
}

template<unsigned DIM>
void SidekickForce<DIM>::SetSupercontractileLineTensionParameter(double supercontractileLineTensionParameter)
{
    mSupercontractileLineTensionParameter = supercontractileLineTensionParameter;
}

template<unsigned DIM>
void SidekickForce<DIM>::SetNumStripes(unsigned numStripes)
{
    mNumStripes = numStripes;
}

template<unsigned DIM>
void SidekickForce<DIM>::OutputForceParameters(out_stream& rParamsFile)
{
    *rParamsFile << "\t\t\t<HomotypicLineTensionParameter>" << mHomotypicLineTensionParameter << "</HomotypicLineTensionParameter> \n";
    *rParamsFile << "\t\t\t<HeterotypicLineTensionParameter>" << mHeterotypicLineTensionParameter << "</HeterotypicLineTensionParameter> \n";
    *rParamsFile << "\t\t\t<SupercontractileLineTensionParameter>" << mSupercontractileLineTensionParameter << "</SupercontractileLineTensionParameter> \n";
    *rParamsFile << "\t\t\t<NumStripes>" << mNumStripes << "</NumStripes> \n";

    FarhadifarForce<DIM>::OutputForceParameters(rParamsFile);
}

// Explicit instantiation
template class SidekickForce<1>;
template class SidekickForce<2>;
template class SidekickForce<3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(SidekickForce)
