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

#ifndef NagaiHondaMultipleLabelsFORCE_HPP_
#define NagaiHondaMultipleLabelsFORCE_HPP_

#include "ChasteSerialization.hpp"
#include <boost/serialization/base_object.hpp>

#include "NagaiHondaForce.hpp"

#include <iostream>

/**
 * A force class for use in vertex-based simulations, based on a model
 * model proposed by T. Nagai and H. Honda ("A dynamic cell model for
 * the formation of epithelial tissues", Philosophical Magazine Part B
 * 81:699-719) to include differential adhesion between normal and
 * labelled cells. To include differential adhesion we override the
 * GetAdhesionParameter() method.
 *
 * Each of the model parameter member variables are rescaled such that
 * mDampingConstantNormal takes the default value 1, whereas Nagai and
 * Honda (who denote the parameter by nu) take the value 0.01.
 */
template<unsigned DIM>
class NagaiHondaMultipleLabelsForce : public NagaiHondaForce<DIM>
{
private:

    /**
     * Whether to use a line tension term that varies as a negative exponential function
     * of the interface length.
     * Takes the default value false (corresponding to a linear line tension, as used by Farhadifar et al.)
     */
    bool mUseExponentialLineTension;

    /**
     * Adhesion parameter for cell edges on the boundary of the population.
     * Takes the default value 1.0.
     */
    double mCellBoundaryAdhesionParameter;

    /**
     * Adhesion parameter for edges shared by two cells whose labels have the same colour.
     * Takes the default value 1.0.
     */
    double mHomotypicCellAdhesionParameter;

    /**
     * Adhesion parameter for edges shared by two cells whose labels have different colours.
     * Takes the default value 1.0.
     */
    double mHeterotypicCellAdhesionParameter;

    /**
     * Parameter determining the scaling of the adhesion energy.
     * Takes the default value 1.0.
     */
    double mLambdaParameter;

    /**
     * Helper member variable that stores the number of distinct cell labels
     * present in the population.
     * Defaults to UNSIGNED_UNSET and is set by calling the method ComputeNumLabelledColours().
     */
    unsigned mNumLabelledColours;

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
        archive & boost::serialization::base_object<NagaiHondaForce<DIM> >(*this);
        archive & mUseExponentialLineTension;
        archive & mCellBoundaryAdhesionParameter;
        archive & mHomotypicCellAdhesionParameter;
        archive & mHeterotypicCellAdhesionParameter;
        archive & mLambdaParameter;
        archive & mNumLabelledColours;
    }

    /**
     * Helper method to compute the value of mNumLabelledColours.
     *
     * This method is called the first time that GetAdhesionParameter() is called.
     * We thus assume that no new cell labels are applied after this call.
     *
     * @param rVertexCellPopulation reference to the cell population
     */
    void ComputeNumLabelledColours(VertexBasedCellPopulation<DIM>& rVertexCellPopulation);

public:

    /**
     * Constructor.
     */
    NagaiHondaMultipleLabelsForce();

    /**
     * Destructor.
     */
    virtual ~NagaiHondaMultipleLabelsForce();

    /**
     * Get the value of #mUseExponentialLineTension.
     *
     * @return mUseExponentialLineTension.
     */
    bool GetUseExponentialLineTension();

    /**
     * Set the value of #mUseExponentialLineTension.
     *
     * @param useExponentialLineTension whether to use an exponential line tension term.
     */
    void SetUseExponentialLineTension(bool useExponentialLineTension);

    /**
     * Overridden AddForceContribution() method.
     *
     * @param rCellPopulation reference to the cell population
     */
    virtual void AddForceContribution(AbstractCellPopulation<DIM>& rCellPopulation);

    /**
     * @return the difference in label colours between the two cells sharing the edge between two given nodes.
     * If the edge belongs to one element then the cell is on the boundary of the tissue; in this case,
     * the method returns UNSIGNED_UNSET.
     *
     * @param pNodeA one node
     * @param pNodeB the other node
     * @param rVertexCellPopulation reference to the cell population
     */
    unsigned GetDifferenceInLabelsAcrossEdge(Node<DIM>* pNodeA, Node<DIM>* pNodeB, VertexBasedCellPopulation<DIM>& rVertexCellPopulation);

    /**
     * Overridden GetAdhesionParameter() method.
     *
     * Get the adhesion parameter for the edge between two given nodes. Depends
     * on the type of cells attached to the elements.
     *
     * @param pNodeA one node
     * @param pNodeB the other node
     * @param rVertexCellPopulation reference to the cell population
     *
     * @return the adhesion parameter for this edge.
     */
    virtual double GetAdhesionParameter(Node<DIM>* pNodeA, Node<DIM>* pNodeB, VertexBasedCellPopulation<DIM>& rVertexCellPopulation);

    /**
     * @return mCellBoundaryAdhesionParameter
     */
    double GetCellBoundaryAdhesionParameter();

    /**
     * @return mHomotypicCellAdhesionParameter
     */
    double GetHomotypicCellAdhesionParameter();

    /**
     * @return mHeterotypicCellAdhesionParameter
     */
    double GetHeterotypicCellAdhesionParameter();

    /**
     * @return mLambdaParameter
     */
    double GetLambdaParameter();

    /**
     * @return #mNumLabelledColours.
     */
    unsigned GetNumLabelledColours();

    /**
     * Set mCellBoundaryAdhesionParameter.
     *
     * @param adhesionEnergyParameterForCellOnBoundary the new value of mCellBoundaryAdhesionParameter
     */
    void SetCellBoundaryAdhesionParameter(double adhesionEnergyParameterForCellOnBoundary);

    /**
     * Set mHomotypicCellAdhesionParameter.
     *
     * @param labelledCellLabelledCellAdhesionEnergyParameter the new value of mHomotypicCellAdhesionParameter
     */
    void SetHomotypicCellAdhesionParameter(double labelledCellLabelledCellAdhesionEnergyParameter);

    /**
     * Set mHeterotypicCellAdhesionParameter.
     *
     * @param labelledCellCellAdhesionEnergyParameter the new value of mHeterotypicCellAdhesionParameter
     */
    void SetHeterotypicCellAdhesionParameter(double labelledCellCellAdhesionEnergyParameter);

    /**
     * Set mLambdaParameter.
     *
     * @param lambdaParameter the new value of mLambdaParameter
     */
    void SetLambdaParameter(double lambdaParameter);

    /**
     * Overridden OutputForceParameters() method.
     *
     * @param rParamsFile the file stream to which the parameters are output
     */
    void OutputForceParameters(out_stream& rParamsFile);
};

#include "SerializationExportWrapper.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(NagaiHondaMultipleLabelsForce)

#endif /*NagaiHondaMultipleLabelsFORCE_HPP_*/
