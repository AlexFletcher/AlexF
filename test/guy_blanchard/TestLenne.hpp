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

#ifndef TestLenne_HPP_
#define TestLenne_HPP_

#include <cxxtest/TestSuite.h>
#include "CheckpointArchiveTypes.hpp"
#include "AbstractCellBasedTestSuite.hpp"
#include "ToroidalHoneycombVertexMeshGenerator.hpp"
#include "HoneycombVertexMeshGenerator.hpp"
#include "CellsGenerator.hpp"
#include "NoCellCycleModel.hpp"
#include "CellLabel.hpp"
#include "VertexBasedCellPopulation.hpp"
#include "CellLabelWriter.hpp"
#include "OffLatticeSimulation.hpp"
#include "SmartPointers.hpp"
#include "FakePetscSetup.hpp"
#include "LenneForce.hpp"
#include "SimpleTargetAreaModifier.hpp"


class TestLenne : public AbstractCellBasedTestSuite
{
public:

    void TestVertexBasedDifferentialAdhesionSimulation() throw (Exception)
    {
        // Create regular mesh
        //ToroidalHoneycombVertexMeshGenerator generator(8, 6);
        //Toroidal2dVertexMesh* p_mesh = generator.GetToroidalMesh();

        unsigned MeshWidth = 14;
        unsigned MeshHeight = 10;
        HoneycombVertexMeshGenerator generator(MeshWidth, MeshHeight);
        MutableVertexMesh<2,2>* p_mesh = generator.GetMesh();

        //\guy 02/02
        // This command specifies the threshold vertex separation for a T1 swap to be triggered
        p_mesh->SetCellRearrangementThreshold(0.1);

        //\guy 02/02
        // This command tells the mesh to check for and deal with internal vertex/edge intersections
        // at each time step (note: this is usually not done, as it slows things down a lot)
        //p_mesh->SetCheckForInternalIntersections(true);

        // Create some differentiated cells
        std::vector<CellPtr> cells;
        CellsGenerator<NoCellCycleModel, 2> cells_generator;
        cells_generator.GenerateBasic(cells, p_mesh->GetNumElements());

        // Create cell population
        VertexBasedCellPopulation<2> cell_population(*p_mesh, cells);
        cell_population.AddCellWriter<CellLabelWriter>();

        // Create simulation
        OffLatticeSimulation<2> simulator(cell_population);
        simulator.SetOutputDirectory("TestLenne");
        simulator.SetDt(0.001);
        simulator.SetSamplingTimestepMultiple(10);
        simulator.SetEndTime(100.0);

        // Create force law
        MAKE_PTR(LenneForce<2>, p_force);
        p_force->SetAreaElasticityParameter(20.0);
	p_force->SetPerimeterContractilityParameter(0.0);
        p_force->SetLineTensionParameter(1.0);
        p_force->SetBoundaryLineTensionParameter(0.5);

        simulator.AddForce(p_force);

        // Pass in a target area modifier (needed, but not used)
        MAKE_PTR(SimpleTargetAreaModifier<2>, p_growth_modifier);
        simulator.AddSimulationModifier(p_growth_modifier);

        // Run simulation
        simulator.Solve();
    }
};

#endif /*TESTRUNNINGDIFFERENTIALADHESIONSIMULATIONSTUTORIAL_HPP_*/
