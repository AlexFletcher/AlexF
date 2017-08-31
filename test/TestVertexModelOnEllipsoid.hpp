
#ifndef TESTVERTEXMODELONELLIPSOID_HPP_
#define TESTVERTEXMODELONELLIPSOID_HPP_

#include "AbstractCellBasedTestSuite.hpp"

#include "GeodesicSphere23Generator.hpp"
#include "MonolayerVertexMeshGenerator.hpp"

#include "CellsGenerator.hpp"
#include "TransitCellProliferativeType.hpp"
#include "UniformG1GenerationalCellCycleModel.hpp"
#include "VertexBasedCellPopulation.hpp"

#include "SmartPointers.hpp"
#include "GeneralMonolayerVertexMeshForce.hpp"
#include "OffLatticeSimulation.hpp"

#include <string>

#include "FakePetscSetup.hpp"

class TestVertexModelOnEllipsoid : public AbstractCellBasedTestSuite
{
public:

    void TestEllipsoid() throw(Exception)
    {
	static const double target_area = 1;
	static const double end_time = 10;

        const std::string output_filename = "TestVertexModelOnEllipsoid";
        GeodesicSphere23Generator builder;
        builder.SubDivide();
        builder.SubDivide();

        MutableVertexMesh<2, 3>* p_dual_mesh = builder.GetDual();
        VertexMeshWriter<2, 3> Writer(output_filename, "Geodesic_Dual", false);
        Writer.WriteVtkUsingMesh(*p_dual_mesh);

        const unsigned radius = sqrt(p_dual_mesh->GetNumElements() * target_area / 4 / M_PI);
        MonolayerVertexMeshGenerator sBuilder;
        MutableVertexMesh<3, 3>* p_mesh = sBuilder.MakeSphericalMesh33(p_dual_mesh, 5, 0.5);
        sBuilder.WriteVtk(output_filename, "InitialMesh");

        std::vector<CellPtr> cells;
        MAKE_PTR(TransitCellProliferativeType, p_transit_type);
        CellsGenerator<UniformG1GenerationalCellCycleModel, 3> cells_generator;
        cells_generator.GenerateBasicRandom(cells, p_mesh->GetNumElements(), p_transit_type);
        VertexBasedCellPopulation<3> cell_population(*p_mesh, cells);

        OffLatticeSimulation<3> simulator(cell_population);
        simulator.SetOutputDirectory(output_filename);
        simulator.SetSamplingTimestepMultiple(10);
        simulator.SetEndTime(end_time);

        MAKE_PTR(GeneralMonolayerVertexMeshForce, p_force3);
        p_force3->SetApicalParameters(20, 20, 0.7);
        p_force3->SetBasalParameters(20, 20, 0.7);
        p_force3->SetLateralParameter(8);
        p_force3->SetVolumeParameters(350, 1);//(350,2)
        simulator.AddForce(p_force3);
        // MAKE_PTR(HorizontalStretchForce<3>, p_force2);
        // p_force2->SetForceMagnitude(1.0);
        // p_force2->SetRelativeWidth(0.15);
        // simulator.AddForce(p_force2);

        simulator.Solve();
    }
};

#endif /*TESTVERTEXMODELONELLIPSOID_HPP_*/

