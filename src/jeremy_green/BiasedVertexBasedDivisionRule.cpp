
#include "BiasedVertexBasedDivisionRule.hpp"
#include "RandomNumberGenerator.hpp"

template <unsigned SPACE_DIM>
c_vector<double, SPACE_DIM> BiasedVertexBasedDivisionRule<SPACE_DIM>::CalculateCellDivisionVector(
    CellPtr pParentCell, VertexBasedCellPopulation<SPACE_DIM>& rCellPopulation)
{
    // Generate an angle of division from the von Mises distribution with mean mu = 0 and concentration k = 10
    double angle = GenerateVonMisesRandomVariate(0, 10);

    c_vector<double, SPACE_DIM> random_vector;
    random_vector(0) = cos(angle);
    random_vector(1) = sin(angle);

    return random_vector;
}

template <unsigned SPACE_DIM>
double BiasedVertexBasedDivisionRule<SPACE_DIM>::GenerateVonMisesRandomVariate(double mu, double k)
{
    double result = 0.0;

    double a = 1.0 + sqrt(1 + 4.0 * (k * k));
    double b = (a - sqrt(2.0 * a))/(2.0 * k);
    double r = (1.0 + b * b)/(2.0 * b);

    while (1)
    {
        double U1 = RandomNumberGenerator::Instance()->ranf();
        double z = cos(M_PI * U1);
        double f = (1.0 + r * z)/(r + z);
        double c = k * (r - f);
        double U2 = RandomNumberGenerator::Instance()->ranf();

        if (c * (2.0 - c) - U2 > 0.0)
        {
            double U3 = RandomNumberGenerator::Instance()->ranf();
            double sign = 0.0;
            if (U3 - 0.5 < 0.0)
            {
                sign = -1.0;
            }
            if (U3 - 0.5 > 0.0)
            {
                sign = 1.0;
            }
            result = sign * std::acos(f) + mu;
            while (result >= 2.0 * M_PI)
            {
                result -= 2.0 * M_PI;
            }
            break;
        }
        else
        {
            if (log(c/U2) + 1.0 - c >= 0.0)
            {
                double U3 = RandomNumberGenerator::Instance()->ranf();
                double sign = 0.0;
                if (U3 - 0.5 < 0.0)
                {
                    sign = -1.0;
                }
                if (U3 - 0.5 > 0.0)
                {
                    sign = 1.0;
                }
                result = sign * std::acos(f) + mu;
                while (result >= 2.0 * M_PI)
                {
                    result -= 2.0 * M_PI;
                }
                break;
            }
        }
    }
    return result;
}


// Explicit instantiation
template class BiasedVertexBasedDivisionRule<1>;
template class BiasedVertexBasedDivisionRule<2>;
template class BiasedVertexBasedDivisionRule<3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(BiasedVertexBasedDivisionRule)
