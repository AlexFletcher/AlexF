
#include "OffTissueAxisVertexBasedDivisionRule.hpp"

///\todo should this be 30 degrees offset from the long axis or the short axis?

template <unsigned SPACE_DIM>
c_vector<double, SPACE_DIM> OffTissueAxisVertexBasedDivisionRule<SPACE_DIM>::CalculateCellDivisionVector(
    CellPtr pParentCell,
    VertexBasedCellPopulation<SPACE_DIM>& rCellPopulation)
{
    // Use an acceptance-rejection method to generate a random angle drawn from the
    // normal distribution N(mean, sigma^2) truncated to the interval [lower, upper]
    // (see http://dx.doi.org/10.1111/rssb.12162)
    double lower = 0;
    double upper = M_PI;
    double mu = M_PI/6.0;
    double sigma = sqrt(M_PI/18.0);
    double l = (lower - mu)/sigma;
    double u = (upper - mu)/sigma;
    double x = RandomNumberGenerator::Instance()->StandardNormalRandomDeviate();
    while (x < l || x > u)
    {
        x = RandomNumberGenerator::Instance()->StandardNormalRandomDeviate();
    }
    double theta = mu + sigma*x;

    // Randomly add or subtract this angle to the axis of tissue stretch, which is assumed to be parallel to the x axis
    double new_angle = -theta + (RandomNumberGenerator::Instance()->ranf() > 0.5)*2.0*theta;

    c_vector<double, SPACE_DIM> division_axis;
    division_axis[0] = cos(new_angle);
    division_axis[1] = sin(new_angle);

    return division_axis;
}

// Explicit instantiation
template class OffTissueAxisVertexBasedDivisionRule<1>;
template class OffTissueAxisVertexBasedDivisionRule<2>;
template class OffTissueAxisVertexBasedDivisionRule<3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(OffTissueAxisVertexBasedDivisionRule)
