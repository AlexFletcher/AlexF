#ifndef OFFTISSUEAXISVERTEXBASEDDIVISIONRULE_HPP_
#define OFFTISSUEAXISVERTEXBASEDDIVISIONRULE_HPP_

#include "ChasteSerialization.hpp"
#include <boost/serialization/base_object.hpp>

#include "AbstractVertexBasedDivisionRule.hpp"
#include "VertexBasedCellPopulation.hpp"

// Forward declaration prevents circular include chain
template<unsigned SPACE_DIM> class VertexBasedCellPopulation;
template<unsigned SPACE_DIM> class AbstractVertexBasedDivisionRule;

template<unsigned SPACE_DIM>
class OffTissueAxisVertexBasedDivisionRule : public AbstractVertexBasedDivisionRule<SPACE_DIM>
{
private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & archive, const unsigned int version)
    {
        archive & boost::serialization::base_object<AbstractVertexBasedDivisionRule<SPACE_DIM> >(*this);
    }

public:

    OffTissueAxisVertexBasedDivisionRule()
    {
    }

    virtual ~OffTissueAxisVertexBasedDivisionRule()
    {
    }

    virtual c_vector<double, SPACE_DIM> CalculateCellDivisionVector(CellPtr pParentCell,
        VertexBasedCellPopulation<SPACE_DIM>& rCellPopulation);
};

#include "SerializationExportWrapper.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(OffTissueAxisVertexBasedDivisionRule)

#endif // OFFOffTissueAxisVertexBasedDivisionRule_HPP_
