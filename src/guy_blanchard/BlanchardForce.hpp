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

#ifndef FARHADIFARFORCEWITHSUPERCONTRACTILITYFORCE_HPP_
#define FARHADIFARFORCEWITHSUPERCONTRACTILITYFORCE_HPP_

#include "ChasteSerialization.hpp"
#include <boost/serialization/base_object.hpp>
#include "FarhadifarForce.hpp"
#include <iostream>

/**
 * A force class for use in vertex-based simulations. This force is based on the
 * energy function proposed by Farhadifar et al (Curr. Biol., 2007, 17, 2095-2104)
 * with the following modifications:
 *
 * 1. We allow for the line tension parameter to take distinct values for cell-cell
 * interfaces between cells whose 'stripe identities' are the same, or differ by one,
 * or differ by two, or for 'boundary' interfaces. (Here, stripe identity is a value
 * from 1 to 4, which is stored in each cell's CellData property as the item "stripe".)
 */
template<unsigned DIM>
class BlanchardForce : public FarhadifarForce<DIM>
{
private:

    /**
     * Line tension parameter for cell edges on the boundary of the population.
     * Defaults to 1.0.
     */
    double mCellBoundaryAdhesionParameter;

    /**
     * Line tension parameter for edges shared by two cells with the same stripe identity.
     * Defaults to 1.0.
     */
    double mHomotypicLineTensionParameter;

    /**
     * Line tension parameter for edges shared by two cells whose stripes identities differ by one.
     * Defaults to 1.0.
     */
    double mHeterotypicLineTensionParameter;

    /**
     * Line tension parameter for edges shared by two cells whose stripes identities differ by two.
     * Defaults to 1.0.
     */
    double mSupercontractileLineTensionParameter;

    /**
     * Helper member variable that stores the number of distinct cell stripes
     * present in the population.
     * Defaults to 4.
     */
    unsigned mNumStripes;

    /**
     * Whether to have the line tension parameter dependent on the total lengths of 'boundary interfaces'
     * experienced by a cell.
     * Defaults to false.
     */
    bool mUseCombinedInterfacesForLineTension;

    /**
     * TODO
     * Defaults to false.
     */
    bool mUseDistinctStripeMismatchesForCombinedInterfaces;

    friend class boost::serialization::access;
    /**
     * Boost Serialization method for archiving/checkpointing.
     * Archives the object and its member variables.
     *
     * @param archive  The boost archive.
     * @param version  The current version of this class.
     */
    template<class Archive>
    void serialize(Archive & archive, const unsigned int version)
    {
        archive & boost::serialization::base_object<FarhadifarForce<DIM> >(*this);
        archive & mHomotypicLineTensionParameter;
        archive & mHeterotypicLineTensionParameter;
        archive & mSupercontractileLineTensionParameter;
        archive & mNumStripes;
        archive & mUseCombinedInterfacesForLineTension;
        archive & mUseDistinctStripeMismatchesForCombinedInterfaces;
    }

    /**
     * \todo
     *
     * @param pNodeA one node
     * @param pNodeB the other node
     * @param elemIndex index of an element containing the node
     * @param cell1StripeIdentity stripe identity of one cell whose element contains the node
     * @param cell2StripeIdentity stripe identity of another cell whose element contains the node
     * @param rVertexCellPopulation reference to the cell population
     *
     * @return the line tension parameter for this edge.
     */
    double GetCombinedInterfaceLength(Node<DIM>* pNode,
                                      unsigned elemIndex,
                                      unsigned cell1StripeIdentity,
                                      unsigned cell2StripeIdentity,
                                      VertexBasedCellPopulation<DIM>& rVertexCellPopulation);

    /**
     * Get a scaling factor for the line tension parameter for the edge between two given nodes.
     * Only called if mUseCombinedInterfacesForLineTension is set to true.
     *
     * @param pNodeA one node
     * @param pNodeB the other node
     * @param element1Index index of one of the elements sharing the edge between pNodeA and pNodeB
     * @param element2Index index of the other element sharing the edge between pNodeA and pNodeB
     * @param cell1StripeIdentity stripe identity of the first element
     * @param cell2StripeIdentity stripe identity of the second element
     * @param rVertexCellPopulation reference to the cell population
     *
     * @return the line tension parameter for this edge.
     */
    double GetCombinedInterfaceScaleFactor(Node<DIM>* pNodeA,
                                           Node<DIM>* pNodeB,
                                           unsigned element1Index,
                                           unsigned element2Index,
                                           unsigned cell1StripeIdentity,
                                           unsigned cell2StripeIdentity,
                                           VertexBasedCellPopulation<DIM>& rVertexCellPopulation);

public:

    /**
     * Constructor.
     */
    BlanchardForce();

    /**
     * Destructor.
     */
    ~BlanchardForce()
    {}

    /**
     * Get the line tension parameter for the edge between two given nodes.
     *
     * @param pNodeA one node
     * @param pNodeB the other node
     * @param rVertexCellPopulation reference to the cell population
     *
     * @return the line tension parameter for this edge.
     */
    double GetLineTensionParameter(Node<DIM>* pNodeA, Node<DIM>* pNodeB, VertexBasedCellPopulation<DIM>& rVertexCellPopulation);

    /**
     * Set mHomotypicLineTensionParameter.
     *
     * @param homotypicLineTensionParameter the new value of mHomotypicLineTensionParameter
     */
    void SetHomotypicLineTensionParameter(double homotypicLineTensionParameter);

    /**
     * Set mHeterotypicLineTensionParameter.
     *
     * @param heterotypicLineTensionParameter the new value of mHeterotypicLineTensionParameter
     */
    void SetHeterotypicLineTensionParameter(double heterotypicLineTensionParameter);

    /**
     * Set mSupercontractileLineTensionParameter.
     *
     * @param supercontractileLineTensionParameter the new value of mSupercontractileLineTensionParameter
     */
    void SetSupercontractileLineTensionParameter(double supercontractileLineTensionParameter);

    /**
     * Set mNumStripes.
     *
     * @param numStripes the new value of mNumStripes
     */
    void SetNumStripes(unsigned numStripes);

    /**
     * Set mUseCombinedInterfacesForLineTension.
     *
     * @param useCombinedInterfaceLineTension the new value of mUseCombinedInterfacesForLineTension
     */
    void SetUseCombinedInterfacesForLineTension(bool useCombinedInterfaceLineTension);

    /**
     * Set mUseDistinctStripeMismatchesForCombinedInterfaces.
     *
     * @param useDistinctStripeMismatchesForCombinedInterfaces the new value of mUseDistinctStripeMismatchesForCombinedInterfaces
     */
    void SetUseDistinctStripeMismatchesForCombinedInterfaces(bool useDistinctStripeMismatchesForCombinedInterfaces);

    /**
     * Overridden OutputForceParameters() method.
     *
     * @param rParamsFile the file stream to which the parameters are output
     */
    void OutputForceParameters(out_stream& rParamsFile);
};

#include "SerializationExportWrapper.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(BlanchardForce)

#endif /*FARHADIFARFORCEWITHSUPERCONTRACTILITYFORCE_HPP_*/
