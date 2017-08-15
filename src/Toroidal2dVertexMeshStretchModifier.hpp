#ifndef TOROIDAL2DVERTEXMESHSTRETCHMODIFIER_HPP_
#define TOROIDAL2DVERTEXMESHSTRETCHMODIFIER_HPP_

#include "ChasteSerialization.hpp"
#include <boost/serialization/base_object.hpp>

#include "AbstractCellBasedSimulationModifier.hpp"

class Toroidal2dVertexMeshStretchModifier : public AbstractCellBasedSimulationModifier<2,2>
{
private:

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & archive, const unsigned int version)
    {
        archive & boost::serialization::base_object<AbstractCellBasedSimulationModifier<2,2> >(*this);
        archive & mSpeed;
    }

    double mSpeed;

public:

    Toroidal2dVertexMeshStretchModifier();
    virtual ~Toroidal2dVertexMeshStretchModifier();
    virtual void UpdateAtEndOfTimeStep(AbstractCellPopulation<2,2>& rCellPopulation);
    virtual void SetupSolve(AbstractCellPopulation<2,2>& rCellPopulation, std::string outputDirectory);
    void SetSpeed(double speed);
    void OutputSimulationModifierParameters(out_stream& rParamsFile);
};

#include "SerializationExportWrapper.hpp"
CHASTE_CLASS_EXPORT(Toroidal2dVertexMeshStretchModifier)

#endif /*TOROIDAL2DVERTEXMESHSTRETCHMODIFIER_HPP_*/
