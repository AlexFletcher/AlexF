
#ifndef FARHADIFARFORCEWITHGROWINGTARGETAREAS_HPP_
#define FARHADIFARFORCEWITHGROWINGTARGETAREAS_HPP_

#include "ChasteSerialization.hpp"
#include <boost/serialization/base_object.hpp>
#include "Exception.hpp"

#include "AbstractForce.hpp"
#include "VertexBasedCellPopulation.hpp"

#include <iostream>

/**
 * \todo Document class
 */
template<unsigned DIM>
class FarhadifarForceWithGrowingTargetAreas : public AbstractForce<DIM>
{
private:

    friend class boost::serialization::access;
    /**
     * Boost Serialization method for archiving/checkpointing.
     * Archives the object and its member variables.
     *
     * @param archive  The boost archive.
     * @param version  The current version of this class.
     */
    template<class Archive>
    void serialize(Archive & archive, const unsigned int version)
    {
        archive & boost::serialization::base_object<AbstractForce<DIM> >(*this);
        archive & mAreaElasticityParameter;
        archive & mPerimeterContractilityParameter;
        archive & mLineTensionParameter;
    }

protected:

    /**
     * The strength of the area term in the model. Corresponds to K_alpha in Farhadifar's paper.
     */
    double mAreaElasticityParameter;

    /**
     * The strength of the perimeter term in the model. Corresponds to Gamma_alpha in Farhadifar's paper.
     */
    double mPerimeterContractilityParameter;

    /**
     * The strength of the line tension term in the model. Lambda_{i,j} in Farhadifar's paper.
     */
    double mLineTensionParameter;


public:

    /**
     * Constructor.
     */
    FarhadifarForceWithGrowingTargetAreas();

    /**
     * Destructor.
     */
    virtual ~FarhadifarForceWithGrowingTargetAreas();

    /**
     * Overridden AddForceContribution() method.
     *
     * Calculates the force on each node in the vertex-based cell population based on the energy function
     * Farhadifar's model.
     *
     * @param rCellPopulation reference to the cell population
     */
    virtual void AddForceContribution(AbstractCellPopulation<DIM>& rCellPopulation);

    /**
     * Get the line tension parameter for the edge between two given nodes.
     *
     * @param pNodeA one node
     * @param pNodeB the other node
     * @param rVertexCellPopulation reference to the cell population
     *
     * @return the line tension parameter for this edge.
     */
    virtual double GetLineTensionParameter(Node<DIM>* pNodeA, Node<DIM>* pNodeB, VertexBasedCellPopulation<DIM>& rVertexCellPopulation);

    /**
     * @return mAreaElasticityParameter
     */
    double GetAreaElasticityParameter();

    /**
     * @return mPerimeterContractilityParameter
     */
    double GetPerimeterContractilityParameter();

    /**
     * @return mLineTensionParameter
     */
    double GetLineTensionParameter();

    /**
     * Set mAreaElasticityParameter.
     *
     * @param areaElasticityParameter the new value of mAreaElasticityParameter
     */
    void SetAreaElasticityParameter(double areaElasticityParameter);

    /**
     * Set mPerimeterContractilityParameter.
     *
     * @param perimeterContractilityParameter the new value of perimterContractilityParameter
     */
    void SetPerimeterContractilityParameter(double perimeterContractilityParameter);

    /**
     * Set mLineTensionParameter.
     *
     * @param lineTensionParameter the new value of mLineTensionParameter
     */
    void SetLineTensionParameter(double lineTensionParameter);

    /**
     * Overridden OutputForceParameters() method.
     *
     * @param rParamsFile the file stream to which the parameters are output
     */
    void OutputForceParameters(out_stream& rParamsFile);
};

#include "SerializationExportWrapper.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(FarhadifarForceWithGrowingTargetAreas)

#endif /*FARHADIFARFORCEWITHGROWINGTARGETAREAS_HPP_*/
