
#ifndef BIASEDVERTEXBASEDDIVISIONRULE_HPP_
#define BIASEDVERTEXBASEDDIVISIONRULE_HPP_

#include "ChasteSerialization.hpp"
#include <boost/serialization/base_object.hpp>
#include "AbstractVertexBasedDivisionRule.hpp"
#include "VertexBasedCellPopulation.hpp"
#include "RandomNumberGenerator.hpp"

// Forward declaration prevents circular include chain
template<unsigned SPACE_DIM> class VertexBasedCellPopulation;
template<unsigned SPACE_DIM> class AbstractVertexBasedDivisionRule;

template <unsigned SPACE_DIM>
class BiasedVertexBasedDivisionRule  : public AbstractVertexBasedDivisionRule<SPACE_DIM>
{
private:
    friend class boost::serialization::access;
    /**
     * Serialize the object and its member variables.
     *
     * @param archive the archive
     * @param version the current version of this class
     */
    template<class Archive>
    void serialize(Archive & archive, const unsigned int version)
    {
        archive & boost::serialization::base_object<AbstractVertexBasedDivisionRule<SPACE_DIM> >(*this);
    }

public:

    /**
     * Default constructor.
     */
    BiasedVertexBasedDivisionRule()
    {
    }

    /**
     * Empty destructor.
     */
    virtual ~BiasedVertexBasedDivisionRule()
    {
    }

    /**
     * Overridden CalculateCellDivisionVector() method.
     *
     * @param pParentCell  The cell to divide
     * @param rCellPopulation  The vertex-based cell population
     *
     * @return the division vector.
     */
    virtual c_vector<double, SPACE_DIM> CalculateCellDivisionVector(
        CellPtr pParentCell, VertexBasedCellPopulation<SPACE_DIM>& rCellPopulation);

    /**
     * Generate a random variable from the von Mises distribution with
     * mean mu and concentration k.
     *
     * @param mu the mean of the von Mises distribution
     * @param k the concentration of the von Mises distribution
     *
     * @return a nov Mises random variate
     */
    double GenerateVonMisesRandomVariate(double mu, double k);
};

#include "SerializationExportWrapper.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(BiasedVertexBasedDivisionRule)

#endif // BIASEDVERTEXBASEDDIVISIONRULE_HPP_
