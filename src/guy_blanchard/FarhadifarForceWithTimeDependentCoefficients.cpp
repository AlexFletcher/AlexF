
#include "FarhadifarForceWithTimeDependentCoefficients.hpp"

template<unsigned DIM>
FarhadifarForceWithTimeDependentCoefficients<DIM>::FarhadifarForceWithTimeDependentCoefficients()
   : FarhadifarForce<DIM>()
{
}

template<unsigned DIM>
FarhadifarForceWithTimeDependentCoefficients<DIM>::~FarhadifarForceWithTimeDependentCoefficients()
{
}

template<unsigned DIM>
double FarhadifarForceWithTimeDependentCoefficients<DIM>::GetAreaElasticityParameter()
{
	double area_elasticity_parameter = this->mAreaElasticityParameter;
	double num_rounds = floor(SimulationTime::Instance()->GetTimeStepsElapsed()*SimulationTime::Instance()->GetTimeStep()/50.0);
	double increase_per_round = 0.1;
	area_elasticity_parameter += increase_per_round*num_rounds;
    return area_elasticity_parameter;
}

template<unsigned DIM>
double FarhadifarForceWithTimeDependentCoefficients<DIM>::GetPerimeterContractilityParameter()
{
	double perimeter_contractility_parameter = this->mPerimeterContractilityParameter;
	double num_rounds = floor(SimulationTime::Instance()->GetTimeStepsElapsed()*SimulationTime::Instance()->GetTimeStep()/50.0);
	double increase_per_round = 0.1;
	perimeter_contractility_parameter += increase_per_round*num_rounds;
	return perimeter_contractility_parameter;
}

template<unsigned DIM>
void FarhadifarForceWithTimeDependentCoefficients<DIM>::OutputForceParameters(out_stream& rParamsFile)
{
    FarhadifarForce<DIM>::OutputForceParameters(rParamsFile);
}

// Explicit instantiation
template class FarhadifarForceWithTimeDependentCoefficients<1>;
template class FarhadifarForceWithTimeDependentCoefficients<2>;
template class FarhadifarForceWithTimeDependentCoefficients<3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(FarhadifarForceWithTimeDependentCoefficients)
