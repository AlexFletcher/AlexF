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

#ifndef TESTAXISEXTENSIONSCENARIOS_HPP_
#define TESTAXISEXTENSIONSCENARIOS_HPP_

#include <cxxtest/TestSuite.h>
#include "CheckpointArchiveTypes.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "HoneycombVertexMeshGenerator.hpp"
#include "ToroidalHoneycombVertexMeshGenerator.hpp"
#include "VoronoiVertexMeshGenerator.hpp"
#include "CellsGenerator.hpp"
#include "NoCellCycleModel.hpp"
#include "CellLabel.hpp"
#include "VertexBasedCellPopulation.hpp"
#include "LenneForce.hpp"
#include "NagaiHondaMultipleLabelsForce.hpp"
#include "RandomForce.hpp"
#include "CellLabelWriter.hpp"
#include "OffLatticeSimulation.hpp"
#include "SmartPointers.hpp"
#include "FakePetscSetup.hpp"
#include "SimpleTargetAreaModifier.hpp"

class TestAxisExtensionScenarios : public AbstractCellBasedWithTimingsTestSuite
{
private:

    void RunSimulation(unsigned hypothesis, unsigned geometry, bool jiggleVertices, double endTime)
    {
        // Generate the name of the output directory based on the input arguments to this method
        std::string output_directory = "Test";
        switch (hypothesis)
        {
            case 0:  { output_directory += "LenneModel";                break; }
            case 1:  { output_directory += "PSBModel";                  break; }
            case 2:  { output_directory += "3StripesModel";             break; }
            case 3:  { output_directory += "4StripesModel";             break; }
            case 4:  { output_directory += "3StripesModelLinearEnergy"; break; }
            case 5:  { output_directory += "4StripesModelLinearEnergy"; break; }
            default: { NEVER_REACHED; }
        }
        switch (geometry)
        {
            case 0:  { output_directory += "RegularGeometry"; break; }
            case 1:  { output_directory += "RandomGeometry";  break; }
            case 2:  { output_directory += "LargeGeometry";   break; }
            default: { NEVER_REACHED; }
        }
        if (jiggleVertices) { output_directory += "Jiggling"; }
        else { output_directory += "NoJiggling"; }

        // Generate a suitable vertex mesh, depending on the specified geometry
        MutableVertexMesh<2,2>* p_mesh;
        unsigned num_cells_wide = 14;
        unsigned num_cells_high = 10;
        HoneycombVertexMeshGenerator honeycomb_generator(num_cells_wide, num_cells_high);
        HoneycombVertexMeshGenerator large_honeycomb_generator(3*num_cells_wide, 3*num_cells_high);
        unsigned num_relaxation_steps = 3;
        double average_target_area = 0.5*sqrt(3.0);
        VoronoiVertexMeshGenerator random_generator(num_cells_wide, num_cells_high, num_relaxation_steps, average_target_area);
        switch (geometry)
        {
            case 0: // RegularInitialCondition
            {
                p_mesh = honeycomb_generator.GetMesh();
                break;
            }
            case 1: // RandomInitialCondition
            {
#if BOOST_VERSION < 105200
    EXCEPTION("VoronoiVertexMeshGenerator requires Boost >= 1.52");
#else // BOOST_VERSION >= 105200
                p_mesh = random_generator.GetMesh();
#endif // BOOST_VERSION >= 105200
                break;
            }
            case 2:
            {
                p_mesh = large_honeycomb_generator.GetMesh();
                break;
            }
            default:
                NEVER_REACHED;
        }

        // This command tells the mesh to check for and deal with internal vertex/edge intersections
        // at each time step (note: this is usually not done, as it slows things down a lot)
        p_mesh->SetCheckForInternalIntersections(true);////////

        // Create some non-proliferating cells
        std::vector<CellPtr> cells;
        CellsGenerator<NoCellCycleModel, 2> cells_generator;
        cells_generator.GenerateBasic(cells, p_mesh->GetNumElements());

        MAKE_PTR_ARGS(CellLabel, p_label_1, (10));
        MAKE_PTR_ARGS(CellLabel, p_label_2, (11));
        MAKE_PTR_ARGS(CellLabel, p_label_3, (12));
        MAKE_PTR_ARGS(CellLabel, p_label_4, (13));
        MAKE_PTR_ARGS(CellLabel, p_label_5, (14));
        MAKE_PTR_ARGS(CellLabel, p_label_6, (15));
        MAKE_PTR_ARGS(CellLabel, p_label_7, (16));
        MAKE_PTR_ARGS(CellLabel, p_label_8, (17));
        MAKE_PTR_ARGS(CellLabel, p_label_9, (18));
        MAKE_PTR_ARGS(CellLabel, p_label_10, (19));
        MAKE_PTR_ARGS(CellLabel, p_label_11, (20));
        MAKE_PTR_ARGS(CellLabel, p_label_12, (21));

        if (hypothesis == 0)
        {
            // Don't label any cells
        }
        else if (hypothesis == 1)
        {
            // Label each cell according to its PS
            for (unsigned i=0; i<cells.size(); i++)
            {
                unsigned row = i/num_cells_wide;
                unsigned col = i%num_cells_wide;

                if (row%2 == 0)
                {
                    // In an even row, the first 4 cells belong to PS 1, the next 3 to PS 2, the next 4 to PS 3, the next 3 to PS 4, etc
                    if (col < 4) { cells[i]->AddCellProperty(p_label_1); }
                    else if (col < 7)  { cells[i]->AddCellProperty(p_label_2);  }
                    else if (col < 11) { cells[i]->AddCellProperty(p_label_3);  }
                    else if (col < 14) { cells[i]->AddCellProperty(p_label_4);  }
                    else if (col < 18) { cells[i]->AddCellProperty(p_label_5);  } // these cases only apply for a large geometry
                    else if (col < 21) { cells[i]->AddCellProperty(p_label_6);  }
                    else if (col < 25) { cells[i]->AddCellProperty(p_label_7);  }
                    else if (col < 28) { cells[i]->AddCellProperty(p_label_8);  }
                    else if (col < 32) { cells[i]->AddCellProperty(p_label_9);  }
                    else if (col < 35) { cells[i]->AddCellProperty(p_label_10); }
                    else if (col < 39) { cells[i]->AddCellProperty(p_label_11); }
                    else               { cells[i]->AddCellProperty(p_label_12); } // col < 42
                }
                else
                {
                    // In an odd row, the first 3 cells belong to PS 1, the next 4 to PS 2, the next 3 to PS 3, the next 4 to PS 4, etc
                    if (col < 3)       { cells[i]->AddCellProperty(p_label_1);  }
                    else if (col < 7)  { cells[i]->AddCellProperty(p_label_2);  }
                    else if (col < 10) { cells[i]->AddCellProperty(p_label_3);  }
                    else if (col < 14) { cells[i]->AddCellProperty(p_label_4);  }
                    else if (col < 17) { cells[i]->AddCellProperty(p_label_5);  } // these cases only apply for a large geometry
                    else if (col < 21) { cells[i]->AddCellProperty(p_label_6);  }
                    else if (col < 24) { cells[i]->AddCellProperty(p_label_7);  }
                    else if (col < 28) { cells[i]->AddCellProperty(p_label_8);  }
                    else if (col < 31) { cells[i]->AddCellProperty(p_label_9);  }
                    else if (col < 35) { cells[i]->AddCellProperty(p_label_10); }
                    else if (col < 38) { cells[i]->AddCellProperty(p_label_11); }
                    else               { cells[i]->AddCellProperty(p_label_12); } // col < 42
                }
            }
        }
        else
        {
            // Label each cell to its identity within each PS (with 3 or 4 stripes, depending on the specified hypothesis)
            for (unsigned i=0; i<cells.size(); i++)
            {
                unsigned row = i/num_cells_wide;
                unsigned col = i%num_cells_wide;

                if (row%4 == 0)
                {
                    if ((col%7 == 0) || (col%7 == 4))      { cells[i]->AddCellProperty(p_label_1); }
                    else if ((col%7 == 1) || (col%7 == 5)) { cells[i]->AddCellProperty(p_label_2); }
                    else if ((col%7 == 2) || (col%7 == 6)) { cells[i]->AddCellProperty(p_label_3); }
                    else // col%7 == 3
                    {
                        if ((hypothesis == 2) || (hypothesis == 4)) { cells[i]->AddCellProperty(p_label_3); } // 3 stripes
                        else { cells[i]->AddCellProperty(p_label_4); } // 4 stripes
                    }
                }
                else if (row%4 == 1)
                {
                    if ((col%7 == 0) || (col%7 == 3))      { cells[i]->AddCellProperty(p_label_1); }
                    else if ((col%7 == 1) || (col%7 == 4)) { cells[i]->AddCellProperty(p_label_2); }
                    else if (col%7 == 5)                   { cells[i]->AddCellProperty(p_label_3); }
                    else // col%7 == 2 || col%7 == 6
                    {
                        if ((hypothesis == 2) || (hypothesis == 4)) { cells[i]->AddCellProperty(p_label_3); } // 3 stripes
                        else                 { cells[i]->AddCellProperty(p_label_4); } // 4 stripes
                    }
                }
                else if (row%4 == 2)
                {
                    if ((col%7 == 0) || (col%7 == 4))      { cells[i]->AddCellProperty(p_label_1); }
                    else if ((col%7 == 1) || (col%7 == 5)) { cells[i]->AddCellProperty(p_label_2); }
                    else if (col%7 == 2)                   { cells[i]->AddCellProperty(p_label_3); }
                    else // col%7 == 3 || col%7 == 6
                    {
                        if ((hypothesis == 2) || (hypothesis == 4)) { cells[i]->AddCellProperty(p_label_3); } // 3 stripes
                        else { cells[i]->AddCellProperty(p_label_4); } // 4 stripes
                    }
                }
                else // row%4 == 3
                {
                    if ((col%7 == 0) || (col%7 == 3))      { cells[i]->AddCellProperty(p_label_1); }
                    else if ((col%7 == 1) || (col%7 == 4)) { cells[i]->AddCellProperty(p_label_2); }
                    else if ((col%7 == 2) || (col%7 == 5)) { cells[i]->AddCellProperty(p_label_3); }
                    else // col%7 == 6
                    {
                        if ((hypothesis == 2) || (hypothesis == 4)) { cells[i]->AddCellProperty(p_label_3); } // 3 stripes
                        else { cells[i]->AddCellProperty(p_label_4); } // 4 stripes
                    }
                }
            }
        }

        // Create a cell population that associates the cells with the vertex mesh
        VertexBasedCellPopulation<2> cell_population(*p_mesh, cells);
        cell_population.SetOutputResultsForChasteVisualizer(false);

        // Output cell labels to VTK at each time step for visualization
        cell_population.AddCellWriter<CellLabelWriter>();

        // Create a simulation using the cell population
        OffLatticeSimulation<2> simulator(cell_population);
        simulator.SetOutputDirectory(output_directory);
        simulator.SetEndTime(endTime);

        // Specify a small time step, but only output results to VTK every hour
        double time_step = 0.001;
        unsigned output_time_step_multiple = (unsigned) (0.1/time_step);//(unsigned) (1.0/time_step); ///\todo
        simulator.SetDt(time_step);
        simulator.SetSamplingTimestepMultiple(output_time_step_multiple);

        // Create the appropriate force law(s) for the specified geometry
        if (hypothesis == 0)
        {
            MAKE_PTR(LenneForce<2>, p_lenne_force);
            p_lenne_force->SetAreaElasticityParameter(20.0);
            p_lenne_force->SetPerimeterContractilityParameter(0.0);//0.2);
            p_lenne_force->SetLineTensionParameter(1.0);
            p_lenne_force->SetBoundaryLineTensionParameter(0.5);
            simulator.AddForce(p_lenne_force);
        }
        else
        {
            MAKE_PTR(NagaiHondaMultipleLabelsForce<2>, p_label_force);
            p_label_force->SetNagaiHondaDeformationEnergyParameter(10.0);
            // Could use to keep cells more isotropic. Currently no obvious need.
            p_label_force->SetNagaiHondaMembraneSurfaceEnergyParameter(1.0);

            // Boundary behaves oddly if this is not set, but probably constrains the deformation of the tissue?
            p_label_force->SetCellBoundaryAdhesionParameter(1.0);
            // Reduce heterotypic interfaces with this penalty. This uses a multiplier depending on neighbour label difference.
            p_label_force->SetHeterotypicCellAdhesionParameter(2.0);//1.0
            // No penalty to within cell type interfaces
            p_label_force->SetHomotypicCellAdhesionParameter(0.0);
            // Penalty for all heterotypic interfaces, particularly the shorter they are, to mimic concentration of myosin with reduction in length, driving to neighbour exchange (hopefully). This also needs multiplier as above.
            p_label_force->SetLambdaParameter(2.0); //1.0

            bool use_exponential_line_tension = (hypothesis < 4);
            p_label_force->SetUseExponentialLineTension(use_exponential_line_tension);

            simulator.AddForce(p_label_force);
        }

        if (jiggleVertices)
        {
            MAKE_PTR(RandomForce<2>, p_random_force);
            p_random_force->SetDiffusionConstant(0.0001); //0.001);
            simulator.AddForce(p_random_force);
        }

        // Pass in a target area modifier (needed, but not used)
        MAKE_PTR(SimpleTargetAreaModifier<2>, p_growth_modifier);
        simulator.AddSimulationModifier(p_growth_modifier);

        // Run simulation
        simulator.Solve();
    }

public:

	// We want 4 stripes in a regular geometry, so set 1st argument to 3 and 2nd argument to 0
    void EMOVETHISCAPITALISEDTEXTTORUN_Test24Sep() throw (Exception)
    {
        RunSimulation(3, 0, false, 100.0);
    }

    /**
     * Test 1:
     *
     * This test simulates a tissue in which all interfaces oriented greater than 45 degrees
     * to the horizontal are contractile, as assumed by Rauzi et al (Nature and anisotropy of
     * cortical forces orienting Drosophila tissue morphogenesis, Nature Cell Biology, 10(12),
     * 1401–1410, 2008, doi:10.1038/ncb1798). We start with a regular tessellation of hexagonal
     * cells and we don't include random jiggling of vertices.
     */
    void REMOVETHISCAPITALISEDTEXTTORUN_TestLenneModelRegularGeometryNoJiggling() throw (Exception)
    {
        RunSimulation(0, 0, false, 100.0);
    }

    /**
     * Test 2:
     *
     * This test is identical to test 1, except that now we include random jiggling of vertices.
     * This is implementing through an additional 'diffusion' force that is assumed to act on
     * each vertex.
     */
    void REMOVETHISCAPITALISEDTEXTTORUN_TestLenneModelRegularGeometryJiggling() throw (Exception)
    {
        RunSimulation(0, 0, true, 100.0);
    }

    /**
     * Test 3:
     *
     * This test is identical to test 2, except that we instead start with an irregular cell
     * packing.
     */
    void REMOVETHISCAPITALISEDTEXTTORUN_TestLenneModelRandomGeometryJiggling() throw (Exception)
    {
        RunSimulation(0, 1, true, 200.0);
    }

    /**
     * Test 4:
     *
     * This test drops the Lenne force and replaces it with a 'positive feedback' exponential
     * dependence of the line tension energy in the interface length for interfaces between
     * cells of different parasegements. There is no line tension energy associated with
     * interfaces between cells within the same parasegment. We start with a regular hexagonal
     * array of cells and do not include random jiggling of vertices.
     */
    void REMOVETHISCAPITALISEDTEXTTORUN_TestPSBModelRegularGeometryNoJiggling() throw (Exception)
    {
        RunSimulation(1, 0, false, 200.0);
    }

    /**
     * Test 5:
     *
     * This test is similar to test 4, except that we now introduce 3 'stripes' of cells within
     * each parasegment, and make the line tension energy depend on whether an interface is
     * between two cells of the same stripe or different stripes.
     */
    void REMOVETHISCAPITALISEDTEXTTORUN_Test3StripesModelRegularGeometryNoJiggling() throw (Exception)
    {
        RunSimulation(2, 0, false, 200.0);
    }

    /**
     * Test 6:
     *
     * This test is similar to test 5, except that we now introduce 4 'stripes' of cells within
     * each parasegment.
     */
    void Test4StripesModelRegularGeometryNoJiggling() throw (Exception)
    {
        RunSimulation(3, 0, false, 200.0);
    }

    /**
     * Test 7:
     *
     * This test is similar to test 5, except that we simulate a larger field of cells.
     */
    void REMOVETHISCAPITALISEDTEXTTORUN_Test3StripesModelLargeGeometryNoJiggling() throw (Exception)
    {
        RunSimulation(2, 2, false, 100.0);
    }

    /**
     * Test 8:
     *
     * This test is similar to test 6, except that we simulate a larger field of cells.
     */
    void REMOVETHISCAPITALISEDTEXTTORUN_Test4StripesModelLargeGeometryNoJiggling() throw (Exception)
    {
        RunSimulation(3, 2, false, 100.0);
    }

    /**
     * Test 9:
     *
     * This test is similar to test 5, except that now we include random jiggling of vertices.
     * This is implementing through an additional 'diffusion' force that is assumed to act on
     * each vertex.
     */
    void REMOVETHISCAPITALISEDTEXTTORUN_Test3StripesModelRegularGeometryJiggling() throw (Exception)
    {
        RunSimulation(2, 0, true, 200.0);
    }

    /**
     * Test 10:
     *
     * This test is similar to test 5, except that we use a line tension energy that is linear, rather
     * than exponential, in the interface length (for those interfaces shared by cells of different
     * stripes).
     */
    void REMOVETHISCAPITALISEDTEXTTORUN_Test3StripesModelLinearEnergyRegularGeometryNoJiggling() throw (Exception)
    {
        RunSimulation(4, 0, false, 1.0);
    }

    /**
     * Test 11:
     *
     * This test is similar to test 6, except that we use a line tension energy that is linear, rather
     * than exponential, in the interface length (for those interfaces shared by cells of different
     * stripes).
     */
    void REMOVETHISCAPITALISEDTEXTTORUN_Test4StripesModelLinearEnergyRegularGeometryNoJiggling() throw (Exception)
    {
        RunSimulation(5, 0, false, 100.0);
    }
};

#endif /*TESTAXISEXTENSIONSCENARIOS_HPP_*/
