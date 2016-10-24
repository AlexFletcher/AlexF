
#ifndef MESHBASEDCELLPOPULATIONWITHOUTREMESHING_HPP_
#define MESHBASEDCELLPOPULATIONWITHOUTREMESHING_HPP_

#include "ChasteSerialization.hpp"
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>

#include "MeshBasedCellPopulation.hpp"

template<unsigned DIM>
class MeshBasedCellPopulationWithoutRemeshing : public MeshBasedCellPopulation<DIM>
{
private:

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & archive, const unsigned int version)
    {
        archive & boost::serialization::base_object<MeshBasedCellPopulation<DIM, DIM> >(*this);
    }

public:
    /**
     * Create a new cell population facade from a mesh and collection of cells.
     *
     * There must be precisely 1 cell for each node of the mesh.
     *
     * @param rMesh a mutable tetrahedral mesh
     * @param rCells cells corresponding to the nodes of the mesh
     * @param locationIndices an optional vector of location indices that correspond to real cells
     * @param deleteMesh set to true if you want the cell population to free the mesh memory on destruction
     * @param validate whether to validate the cell population
     */
    MeshBasedCellPopulationWithoutRemeshing(MutableMesh<ELEMENT_DIM, SPACE_DIM>& rMesh,
                                            std::vector<CellPtr>& rCells,
                                            const std::vector<unsigned> locationIndices=std::vector<unsigned>(),
                                            bool deleteMesh=false,
                                            bool validate=true);

    /**
     * Constructor for use by the de-serializer.
     *
     * @param rMesh a mutable tetrahedral mesh.
     */
    MeshBasedCellPopulationWithoutRemeshing(MutableMesh<ELEMENT_DIM, SPACE_DIM>& rMesh);

    /**
     * Destructor.
     */
    virtual ~MeshBasedCellPopulationWithoutRemeshing();

    /**
     * Overridden Update() method.
     *
     * @param hasHadBirthsOrDeaths an unused boolean specifying whether there have been any cell division or removal events
     */
    virtual void Update(bool hasHadBirthsOrDeaths=true);
};

#include "SerializationExportWrapper.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(MeshBasedCellPopulationWithoutRemeshing)

namespace boost
{
namespace serialization
{
/**
 * Serialize information required to construct a MeshBasedCellPopulationWithoutRemeshing.
 */
template<class Archive, unsigned DIM>
inline void save_construct_data(
    Archive & ar, const MeshBasedCellPopulationWithoutRemeshing<DIM> * t, const unsigned int file_version)
{
    // Save data required to construct instance
    const MutableMesh<DIM,DIM>* p_mesh = &(t->rGetMesh());
    ar & p_mesh;
}

/**
 * De-serialize constructor parameters and initialise a MeshBasedCellPopulationWithoutRemeshing.
 * Loads the mesh from separate files.
 */
template<class Archive, unsigned DIM>
inline void load_construct_data(
    Archive & ar, MeshBasedCellPopulationWithoutRemeshing<DIM> * t, const unsigned int file_version)
{
    // Retrieve data from archive required to construct new instance
    MutableMesh<DIM,DIM>* p_mesh;
    ar >> p_mesh;

    // Invoke inplace constructor to initialise instance
    ::new(t)MeshBasedCellPopulationWithoutRemeshing<DIM>(*p_mesh);
}
}
} // namespace

#endif /*MESHBASEDCELLPOPULATIONWITHOUTREMESHING_HPP_*/
