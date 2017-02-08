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

#ifndef TESTNEWGAMMACELLCYCLEMODEL_HPP_
#define TESTNEWGAMMACELLCYCLEMODEL_HPP_

#include <cxxtest/TestSuite.h>

#include "CheckpointArchiveTypes.hpp"
#include "WildTypeCellMutationState.hpp"
#include "TransitCellProliferativeType.hpp"
#include "NewGammaDistributedCellCycleModel.hpp"
#include "VertexBasedCellPopulation.hpp"
#include "OffLatticeSimulation.hpp"
#include "NagaiHondaDifferentialAdhesionForce.hpp"
#include "SmartPointers.hpp"
#include "AbstractCellBasedTestSuite.hpp"

class TestNewGammaCellCycleModel : public AbstractCellBasedTestSuite
{
public:

    void TestNewGammaCellCycleModelTest() throw (Exception)
    {
        double dt = 0.001;

        std::vector<Node<2>*> nodes;
        unsigned num_nodes = 6;
        std::vector<double> angles = std::vector<double>(num_nodes);
        for (unsigned i=0; i<num_nodes; i++)
        {
            angles[i] = M_PI+2.0*M_PI*(double)(i)/(double)(num_nodes);
            nodes.push_back(new Node<2>(i, true, 0.5*cos(angles[i]), 0.5*sin(angles[i])));
        }

        std::vector<VertexElement<2,2>*> elements;
        elements.push_back(new VertexElement<2,2>(0, nodes));

        MutableVertexMesh<2,2> mesh(nodes, elements);
        mesh.SetCellRearrangementThreshold(0.05);

        std::vector<CellPtr> cells;
        MAKE_PTR(WildTypeCellMutationState, p_state);
        MAKE_PTR(TransitCellProliferativeType, p_prolif_type);

        NewGammaDistributedCellCycleModel* p_model = new NewGammaDistributedCellCycleModel();
        p_model->SetDimension(2);
        p_model->SetShapeParameter(1);
        p_model->SetScaleParameter(20/dt);

        CellPtr p_cell(new Cell(p_state, p_model));
        p_cell->SetCellProliferativeType(p_prolif_type);
        p_cell->SetBirthTime(0.0);

        p_cell->GetCellData()->SetItem("target area", 0.5);

        cells.push_back(p_cell);

        VertexBasedCellPopulation<2> cell_population(mesh, cells);

        // Specify what to output from the simulation
        cell_population.SetOutputResultsForChasteVisualizer(false);
        cell_population.SetOutputCellRearrangementLocations(false);
        cell_population.SetOutputCellProliferativeTypes(false);
        cell_population.SetOutputCellCyclePhases(false);
        cell_population.SetOutputCellAges(false);
        cell_population.SetOutputCellVolumes(false);
        cell_population.SetOutputCellVariables(false);
        cell_population.SetOutputCellMutationStates(false);

        // Create a cell-based simulation
        OffLatticeSimulation<2> simulator(cell_population);
        simulator.SetOutputDirectory("TestNewGammaCellCycleModel");
        simulator.SetDt(dt);
        simulator.SetSamplingTimestepMultiple(10);
        simulator.SetEndTime(30);

        MAKE_PTR(NagaiHondaForce<2>, p_force);
        p_force->SetNagaiHondaDeformationEnergyParameter(300); // 100=default value
        p_force->SetNagaiHondaMembraneSurfaceEnergyParameter(10); // default value
        p_force->SetNagaiHondaCellCellAdhesionEnergyParameter(1); // default value
        p_force->SetNagaiHondaCellBoundaryAdhesionEnergyParameter(1); // default value
        simulator.AddForce(p_force);

        simulator.Solve();
    }
};

#endif /*TESTNEWGAMMACELLCYCLEMODEL_HPP_*/
