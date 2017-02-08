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

#ifndef TEST3DVERTEXMODELSIMULATION_HPP_
#define TEST3DVERTEXMODELSIMULATION_HPP_

#include <cxxtest/TestSuite.h>
#include "CheckpointArchiveTypes.hpp"
#include "AbstractCellBasedTestSuite.hpp"
#include "CellsGenerator.hpp"
#include "DifferentiatedCellProliferativeType.hpp"
#include "StochasticDurationCellCycleModel.hpp"
#include "OffLatticeSimulation.hpp"
#include "VertexBasedCellPopulation.hpp"
#include "PlaneBoundaryCondition.hpp"
#include "FakePetscSetup.hpp"
#include "Debug.hpp"
#include "HoneycombVertexMeshGenerator.hpp"
#include "HexagonalPrism3dVertexMeshGenerator.hpp"
#include "ToyCompressionForce.hpp"
#include "ApicalEdgesForce.hpp"

class Test3dVertexModelSimulation : public AbstractCellBasedTestSuite
{
public:

      void xxTestVertexBasedMonolayer() throw (Exception)
      {
          HoneycombVertexMeshGenerator generator(3, 3);
          MutableVertexMesh<2,2>* p_mesh = generator.GetMesh();

          std::vector<CellPtr> cells;
          MAKE_PTR(DifferentiatedCellProliferativeType, p_type);
          CellsGenerator<StochasticDurationCellCycleModel, 2> cells_generator;
          cells_generator.GenerateBasicRandom(cells, p_mesh->GetNumElements(), p_type);

          VertexBasedCellPopulation<2> cell_population(*p_mesh, cells);

          OffLatticeSimulation<2> simulator(cell_population);
          simulator.SetOutputDirectory("TestVertexBasedMonolayer");
          simulator.SetSamplingTimestepMultiple(200);
          simulator.SetEndTime(20.0);

          simulator.Solve();
      }

    void Test3dVertexModelSimulationWithApicalEdgesForce() throw (Exception)
    {
        // Set up an initial mesh
        HexagonalPrism3dVertexMeshGenerator generator(6, 6, 1.0, 2.0);
        MutableVertexMesh<3,3>* p_mesh = generator.GetMesh();

        // Create a vector of cells
        std::vector<CellPtr> cells;
        MAKE_PTR(DifferentiatedCellProliferativeType, p_type);
        CellsGenerator<StochasticDurationCellCycleModel, 2> cells_generator;
        cells_generator.GenerateBasicRandom(cells, p_mesh->GetNumElements(), p_type);

        // Create a cell population object that associates cells with the mesh
        VertexBasedCellPopulation<3> cell_population(*p_mesh, cells);

        // Create a simulation object that evolves the cell population
        OffLatticeSimulation<3> simulation(cell_population);

        // Set the output directory, simulation duration and how often to output results
        simulation.SetOutputDirectory("Test3dVertexModelSimulationWithApicalEdgesForce");
        simulation.SetSamplingTimestepMultiple(50);
        simulation.SetEndTime(10.0);

        // Create a force object and pass it to the simulation
        MAKE_PTR(ApicalEdgesForce<3>, p_force);
        p_force->SetLambda(0.1);
        simulation.AddForce(p_force);

        // Run the simulation
        simulation.Solve();
    }
};

#endif /*TEST3DVERTEXMODELSIMULATION_HPP_*/
