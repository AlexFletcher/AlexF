
#ifndef SLIDINGBOUNDARYCONDITION_HPP_
#define SLIDINGBOUNDARYCONDITION_HPP_

#include "AbstractCellPopulationBoundaryCondition.hpp"

#include "ChasteSerialization.hpp"
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM=ELEMENT_DIM>
class SlidingBoundaryCondition : public AbstractCellPopulationBoundaryCondition<ELEMENT_DIM,SPACE_DIM>
{
private:

    /**
     * A point on the boundary plane.
     */
    c_vector<double, SPACE_DIM> mPointOnPlane;

    /**
     * The outward-facing unit normal vector to the boundary plane.
     */
    c_vector<double, SPACE_DIM> mNormalToPlane;

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
        archive & boost::serialization::base_object<AbstractCellPopulationBoundaryCondition<ELEMENT_DIM, SPACE_DIM> >(*this);
    }

public:

    SlidingBoundaryCondition(AbstractCellPopulation<ELEMENT_DIM, SPACE_DIM>* pCellPopulation,
                             c_vector<double, SPACE_DIM> point,
                             c_vector<double, SPACE_DIM> normal);

    const c_vector<double, SPACE_DIM>& rGetPointOnPlane() const;
    const c_vector<double, SPACE_DIM>& rGetNormalToPlane() const;

    void ImposeBoundaryCondition(const std::map<Node<SPACE_DIM>*,
                                 c_vector<double, SPACE_DIM> >& rOldLocations);

    bool VerifyBoundaryCondition();
    void OutputCellPopulationBoundaryConditionParameters(out_stream& rParamsFile);
};

#include "SerializationExportWrapper.hpp"
EXPORT_TEMPLATE_CLASS_ALL_DIMS(SlidingBoundaryCondition)

namespace boost
{
namespace serialization
{
/**
 * Serialize information required to construct a SlidingBoundaryCondition.
 */
template<class Archive, unsigned ELEMENT_DIM, unsigned SPACE_DIM>
inline void save_construct_data(
    Archive & ar, const SlidingBoundaryCondition<ELEMENT_DIM, SPACE_DIM>* t, const unsigned int file_version)
{
    // Save data required to construct instance
    const AbstractCellPopulation<ELEMENT_DIM, SPACE_DIM>* const p_cell_population = t->GetCellPopulation();
    ar << p_cell_population;

    // Archive c_vectors one component at a time
    c_vector<double, SPACE_DIM> point = t->rGetPointOnPlane();
    for (unsigned i=0; i<SPACE_DIM; i++)
    {
        ar << point[i];
    }
    c_vector<double, SPACE_DIM> normal = t->rGetNormalToPlane();
    for (unsigned i=0; i<SPACE_DIM; i++)
    {
        ar << normal[i];
    }
}

/**
 * De-serialize constructor parameters and initialize a SlidingBoundaryCondition.
 */
template<class Archive, unsigned ELEMENT_DIM, unsigned SPACE_DIM>
inline void load_construct_data(
    Archive & ar, SlidingBoundaryCondition<ELEMENT_DIM, SPACE_DIM>* t, const unsigned int file_version)
{
    // Retrieve data from archive required to construct new instance
    AbstractCellPopulation<ELEMENT_DIM, SPACE_DIM>* p_cell_population;
    ar >> p_cell_population;

    // Archive c_vectors one component at a time
    c_vector<double, SPACE_DIM> point;
    for (unsigned i=0; i<SPACE_DIM; i++)
    {
        ar >> point[i];
    }
    c_vector<double, SPACE_DIM> normal;
    for (unsigned i=0; i<SPACE_DIM; i++)
    {
        ar >> normal[i];
    }

    // Invoke inplace constructor to initialise instance
    ::new(t)SlidingBoundaryCondition<ELEMENT_DIM, SPACE_DIM>(p_cell_population, point, normal);
}
}
} // namespace ...

#endif /*SLIDINGBOUNDARYCONDITION_HPP_*/
