#include "FollicularEpitheliumStretchModifier.hpp"

template<unsigned DIM>
FollicularEpitheliumStretchModifier<DIM>::FollicularEpitheliumStretchModifier()
    : AbstractCellBasedSimulationModifier<DIM>(),
      mApplyExtrinsicPullToAllNodes(true),
      mPinAnteriorMostCells(false),
      mSpeed(1.0),
      mIncreaseStretchOverTime(false)
{
}

template<unsigned DIM>
FollicularEpitheliumStretchModifier<DIM>::~FollicularEpitheliumStretchModifier()
{
}

template<unsigned DIM>
void FollicularEpitheliumStretchModifier<DIM>::UpdateAtEndOfTimeStep(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
{
    double dt = SimulationTime::Instance()->GetTimeStep();

    if (mIncreaseStretchOverTime)
    {
        mSpeed *= (1.0 + 0.1*dt);
    }

    unsigned num_nodes = rCellPopulation.GetNumNodes();

    ChasteCuboid<DIM> bounds = rCellPopulation.rGetMesh().CalculateBoundingBox();
    double x_min = bounds.rGetLowerCorner()[0];
    double x_max = bounds.rGetUpperCorner()[0];
    double width = x_max - x_min;

    if (mApplyExtrinsicPullToAllNodes)
    {
        for (unsigned node_index=0; node_index<num_nodes; node_index++)
        {
            Node<DIM>* p_node = rCellPopulation.GetNode(node_index);
            double scaled_width = p_node->rGetLocation()[0] - x_min;

            if (mPinAnteriorMostCells)
            {
                ///\todo something
            }

            p_node->rGetModifiableLocation()[0] += (scaled_width/width)*mSpeed*dt;
        }
    }
    else
    {
        for (unsigned node_index=0; node_index<num_nodes; node_index++)
        {
            Node<DIM>* p_node = rCellPopulation.GetNode(node_index);
            if (fabs(p_node->rGetLocation()[0] - x_max) < 0.1)
            {
                p_node->rGetModifiableLocation()[0] += mSpeed*dt;
            }
        }
    }
}

template<unsigned DIM>
void FollicularEpitheliumStretchModifier<DIM>::SetupSolve(AbstractCellPopulation<DIM,DIM>& rCellPopulation, std::string outputDirectory)
{
}

template<unsigned DIM>
void FollicularEpitheliumStretchModifier<DIM>::ApplyExtrinsicPullToAllNodes(bool applyExtrinsicPullToAllNodes)
{
    mApplyExtrinsicPullToAllNodes = applyExtrinsicPullToAllNodes;
}

template<unsigned DIM>
void FollicularEpitheliumStretchModifier<DIM>::PinAnteriorMostCells(bool pinAnteriorMostCells)
{
    mPinAnteriorMostCells = pinAnteriorMostCells;
}

template<unsigned DIM>
void FollicularEpitheliumStretchModifier<DIM>::SetSpeed(double speed)
{
    mSpeed = speed;
}

template<unsigned DIM>
void FollicularEpitheliumStretchModifier<DIM>::IncreaseStretchOverTime(bool increaseStretchOverTime)
{
    mIncreaseStretchOverTime = increaseStretchOverTime;
}

template<unsigned DIM>
void FollicularEpitheliumStretchModifier<DIM>::OutputSimulationModifierParameters(out_stream& rParamsFile)
{
    *rParamsFile << "\t\t\t<ApplyExtrinsicPullToAllNodes>" << mApplyExtrinsicPullToAllNodes << "</ApplyExtrinsicPullToAllNodes>\n";
    *rParamsFile << "\t\t\t<PinAnteriorMostCells>" << mPinAnteriorMostCells << "</PinAnteriorMostCells>\n";
    *rParamsFile << "\t\t\t<Speed>" << mSpeed << "</Speed>\n";
    *rParamsFile << "\t\t\t<IncreaseStretchOverTime>" << mIncreaseStretchOverTime << "</IncreaseStretchOverTime>\n";

    AbstractCellBasedSimulationModifier<DIM>::OutputSimulationModifierParameters(rParamsFile);
}

// Explicit instantiation
template class FollicularEpitheliumStretchModifier<1>;
template class FollicularEpitheliumStretchModifier<2>;
template class FollicularEpitheliumStretchModifier<3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(FollicularEpitheliumStretchModifier)
