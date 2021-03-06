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

#include "MyosinSpringForce.hpp"

template<unsigned DIM>
MyosinSpringForce<DIM>::MyosinSpringForce()
   : AbstractForce<DIM>(),
     mMyosinSpringConstant(1.0),
     mMyosinSpringRestLength(1.0),
     mNonMyosinSpringConstant(1.0),
     mNonMyosinSpringRestLength(1.0)
{
}

template<unsigned DIM>
MyosinSpringForce<DIM>::~MyosinSpringForce()
{
}

template<unsigned DIM>
void MyosinSpringForce<DIM>::AddForceContribution(AbstractCellPopulation<DIM>& rCellPopulation)
{
}

template<unsigned DIM>
double MyosinSpringForce<DIM>::GetMyosinSpringConstant()
{
    return mMyosinSpringConstant;
}

template<unsigned DIM>
double MyosinSpringForce<DIM>::GetMyosinSpringRestLength()
{
    return mMyosinSpringRestLength;
}

template<unsigned DIM>
double MyosinSpringForce<DIM>::GetNonMyosinSpringConstant()
{
    return mNonMyosinSpringConstant;
}

template<unsigned DIM>
double MyosinSpringForce<DIM>::GetNonMyosinSpringRestLength()
{
    return mNonMyosinSpringRestLength;
}

template<unsigned DIM>
void MyosinSpringForce<DIM>::SetMyosinSpringConstant(double myosinSpringConstant)
{
    mMyosinSpringConstant = myosinSpringConstant;
}

template<unsigned DIM>
void MyosinSpringForce<DIM>::SetMyosinSpringRestLength(double myosinSpringRestLength)
{
    mMyosinSpringRestLength = myosinSpringRestLength;
}

template<unsigned DIM>
void MyosinSpringForce<DIM>::SetNonMyosinSpringConstant(double nonMyosinSpringConstant)
{
    mNonMyosinSpringConstant = nonMyosinSpringConstant;
}

template<unsigned DIM>
void MyosinSpringForce<DIM>::SetNonMyosinSpringRestLength(double nonMyosinSpringRestLength)
{
    mNonMyosinSpringRestLength = nonMyosinSpringRestLength;
}

template<unsigned DIM>
void MyosinSpringForce<DIM>::OutputForceParameters(out_stream& rParamsFile)
{
    *rParamsFile << "\t\t\t<MyosinSpringConstant>" << mMyosinSpringConstant << "</MyosinSpringConstant>\n";
    *rParamsFile << "\t\t\t<MyosinSpringRestLength>" << mMyosinSpringRestLength << "</MyosinSpringRestLength>\n";
    *rParamsFile << "\t\t\t<NonMyosinSpringConstant>" << mNonMyosinSpringConstant << "</NonMyosinSpringConstant>\n";
    *rParamsFile << "\t\t\t<NonMyosinSpringRestLength>" << mNonMyosinSpringRestLength << "</NonMyosinSpringRestLength>\n";

    // Call method on direct parent class
    AbstractForce<DIM>::OutputForceParameters(rParamsFile);
}

// Explicit instantiation
template class MyosinSpringForce<1>;
template class MyosinSpringForce<2>;
template class MyosinSpringForce<3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(MyosinSpringForce)
