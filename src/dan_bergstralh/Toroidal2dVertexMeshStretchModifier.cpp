#include "Toroidal2dVertexMeshStretchModifier.hpp"
#include "Toroidal2dVertexMesh.hpp"
#include <cassert>

Toroidal2dVertexMeshStretchModifier::Toroidal2dVertexMeshStretchModifier()
    : AbstractCellBasedSimulationModifier<2>(),
      mSpeed(1.0)
{
}

Toroidal2dVertexMeshStretchModifier::~Toroidal2dVertexMeshStretchModifier()
{
}

void Toroidal2dVertexMeshStretchModifier::UpdateAtEndOfTimeStep(AbstractCellPopulation<2,2>& rCellPopulation)
{
    assert(bool(dynamic_cast<Toroidal2dVertexMesh*>(&(rCellPopulation.rGetMesh()))));
    Toroidal2dVertexMesh* p_mesh = static_cast<Toroidal2dVertexMesh*>(&(rCellPopulation.rGetMesh()));

    double dt = SimulationTime::Instance()->GetTimeStep();

    double old_width = p_mesh->GetWidth(0);
    double new_width = old_width + mSpeed*dt;
    p_mesh->SetWidth(0, new_width);

    double old_height = p_mesh->GetWidth(1);
    double new_height = old_height + mSpeed*dt/sqrt(3.0);
    p_mesh->SetWidth(1, new_height);
}

void Toroidal2dVertexMeshStretchModifier::SetupSolve(AbstractCellPopulation<2,2>& rCellPopulation, std::string outputDirectory)
{
}

void Toroidal2dVertexMeshStretchModifier::SetSpeed(double speed)
{
    mSpeed = speed;
}

void Toroidal2dVertexMeshStretchModifier::OutputSimulationModifierParameters(out_stream& rParamsFile)
{
    *rParamsFile << "\t\t\t<Speed>" << mSpeed << "</Speed>\n";
    AbstractCellBasedSimulationModifier<2>::OutputSimulationModifierParameters(rParamsFile);
}

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
CHASTE_CLASS_EXPORT(Toroidal2dVertexMeshStretchModifier)
