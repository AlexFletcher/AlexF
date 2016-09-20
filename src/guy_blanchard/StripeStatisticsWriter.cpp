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

#include "StripeStatisticsWriter.hpp"
#include "AbstractCellPopulation.hpp"
#include "MeshBasedCellPopulation.hpp"
#include "CaBasedCellPopulation.hpp"
#include "NodeBasedCellPopulation.hpp"
#include "PottsBasedCellPopulation.hpp"
#include "VertexBasedCellPopulation.hpp"

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
StripeStatisticsWriter<ELEMENT_DIM, SPACE_DIM>::StripeStatisticsWriter()
    : AbstractCellPopulationWriter<ELEMENT_DIM, SPACE_DIM>("stripestatistics.dat")
{
}

// We neglect boundary edges here
template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void StripeStatisticsWriter<ELEMENT_DIM, SPACE_DIM>::Visit(VertexBasedCellPopulation<SPACE_DIM>* pCellPopulation)
{
    // Make sure the cell population is updated
    pCellPopulation->Update();

    // Initialise helper variables
    double total_num_edges = 0.0;
    double total_edges_length = 0.0;
    double mismatch_one_num_edges = 0.0;
    double mismatch_one_boundary_length = 0.0;
    double mismatch_two_num_edges = 0.0;
    double mismatch_two_boundary_length = 0.0;

    // Iterate over cells
    for (typename AbstractCellPopulation<SPACE_DIM>::Iterator cell_iter = pCellPopulation->Begin();
         cell_iter != pCellPopulation->End();
         ++cell_iter)
    {
        // Find this cell's stripe identity
        unsigned cell_stripe_identity = cell_iter->GetCellData()->GetItem("stripe");

        // Get the set of neighbouring element indices
        unsigned elem_index = pCellPopulation->GetLocationIndexUsingCell(*cell_iter);
        std::set<unsigned> neighbour_elem_indices = pCellPopulation->rGetMesh().GetNeighbouringElementIndices(elem_index);

        // Iterate over these neighbours
        for (std::set<unsigned>::iterator neighbour_iter = neighbour_elem_indices.begin();
             neighbour_iter != neighbour_elem_indices.end();
             ++neighbour_iter)
        {
            // Get the length of the edge shared with this neighbour
            unsigned neighbour_index = *neighbour_iter;
            double edge_length = pCellPopulation->rGetMesh().GetEdgeLength(elem_index, neighbour_index);

            total_edges_length += edge_length;
            total_num_edges += 1.0;

            // Find this neighbour's stripe identity
            CellPtr p_neighbour = pCellPopulation->GetCellUsingLocationIndex(*neighbour_iter);
            unsigned neighbour_stripe_identity = p_neighbour->GetCellData()->GetItem("stripe");

            unsigned num_stripes = 4; ///\todo remove hardcoding
            unsigned mismatch = abs(cell_stripe_identity - neighbour_stripe_identity);
            if (mismatch > num_stripes/2)
            {
                mismatch = num_stripes - mismatch;
            }

            if (mismatch == 1)
            {
                mismatch_one_num_edges += 1.0;
                mismatch_one_boundary_length += edge_length;
            }
            else if (mismatch == 2)
            {
                mismatch_two_num_edges += 1.0;
                mismatch_two_boundary_length += edge_length;
            }
            else
            {
                assert(mismatch == 0);
            }
        }
    }

    // We have counted each cell-cell edge twice
    total_num_edges *= 0.5;
    total_edges_length *= 0.5;
    mismatch_one_num_edges *= 0.5;
    mismatch_one_boundary_length *= 0.5;
    mismatch_two_num_edges *= 0.5;
    mismatch_two_boundary_length *= 0.5;

    *this->mpOutStream << total_num_edges << "\t" << total_edges_length
                       << "\t" << mismatch_one_num_edges << "\t" << mismatch_one_boundary_length
                       << "\t" << mismatch_two_num_edges << "\t" << mismatch_two_boundary_length;
}

// Explicit instantiation
template class StripeStatisticsWriter<1,1>;
template class StripeStatisticsWriter<1,2>;
template class StripeStatisticsWriter<2,2>;
template class StripeStatisticsWriter<1,3>;
template class StripeStatisticsWriter<2,3>;
template class StripeStatisticsWriter<3,3>;

#include "SerializationExportWrapperForCpp.hpp"
// Declare identifier for the serializer
EXPORT_TEMPLATE_CLASS_ALL_DIMS(StripeStatisticsWriter)
