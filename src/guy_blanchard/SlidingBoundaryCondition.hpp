
#ifndef SLIDINGBOUNDARYCONDITION_HPP_
#define SLIDINGBOUNDARYCONDITION_HPP_

#include "AbstractCellPopulationBoundaryCondition.hpp"

#include "ChasteSerialization.hpp"
#include <boost/serialization/base_object.hpp>

class SlidingBoundaryCondition : public AbstractCellPopulationBoundaryCondition<2,2>
{
private:

    double mXMin;
    double mYMin;
    double mXMax;
    double mYMax;

    /** Needed for serialization. */
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
        archive & boost::serialization::base_object<AbstractCellPopulationBoundaryCondition<2,2> >(*this);
        archive & mXMin;
        archive & mYMin;
        archive & mXMax;
        archive & mYMax;
    }

public:

    SlidingBoundaryCondition(AbstractCellPopulation<2, 2>* pCellPopulation,
                             double xMin=-DOUBLE_UNSET,
                             double yMin=-DOUBLE_UNSET,
                             double xMax=DOUBLE_UNSET,
                             double yMax=DOUBLE_UNSET);

    void ImposeBoundaryCondition(const std::map<Node<2>*, c_vector<double, 2> >& rOldLocations);

    bool VerifyBoundaryCondition();
    void OutputCellPopulationBoundaryConditionParameters(out_stream& rParamsFile);
};

#include "SerializationExportWrapper.hpp"
CHASTE_CLASS_EXPORT(SlidingBoundaryCondition)

namespace boost
{
namespace serialization
{
template<class Archive>
inline void save_construct_data(Archive & ar, const SlidingBoundaryCondition* t, const unsigned int file_version)
{
    const AbstractCellPopulation<2,2>* const p_cell_population = t->GetCellPopulation();
    ar << p_cell_population;
}

template<class Archive>
inline void load_construct_data(Archive & ar, SlidingBoundaryCondition* t, const unsigned int file_version)
{
    // Retrieve data from archive required to construct new instance
    AbstractCellPopulation<2,2>* p_cell_population;
    ar >> p_cell_population;

    ::new(t)SlidingBoundaryCondition(p_cell_population);
}
}
}

#endif /*SLIDINGBOUNDARYCONDITION_HPP_*/
