
#ifndef TARGETAREAMODIFIERFORAREABASEDCELLCYCLEMODEL_HPP_
#define TARGETAREAMODIFIERFORAREABASEDCELLCYCLEMODEL_HPP_

#include "ChasteSerialization.hpp"
#include <boost/serialization/base_object.hpp>

#include "AbstractTargetAreaModifier.hpp"
#include "VertexBasedCellPopulation.hpp"

template<unsigned DIM>
class TargetAreaModifierForAreaBasedCellCycleModel : public AbstractTargetAreaModifier<DIM>
{
private:

    /** Needed for serialization. */
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
        archive & boost::serialization::base_object<AbstractTargetAreaModifier<DIM> >(*this);
    }

public:

    /**
     * Default constructor.
     */
    TargetAreaModifierForAreaBasedCellCycleModel();

    /**
     * Destructor.
     */
    virtual ~TargetAreaModifierForAreaBasedCellCycleModel();

    /**
     * Helper method to update the target area property of an individual cell.
     *
     * @param pCell pointer to a cell
     */
    void UpdateTargetAreaOfCell(const CellPtr pCell);

    /**
     * Overridden OutputSimulationModifierParameters() method.
     * Output any simulation modifier parameters to file.
     *
     * @param rParamsFile the file stream to which the parameters are output
     */
    void OutputSimulationModifierParameters(out_stream& rParamsFile);
};

#include "SerializationExportWrapper.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(TargetAreaModifierForAreaBasedCellCycleModel)

#endif /*TARGETAREAMODIFIERFORAREABASEDCELLCYCLEMODEL_HPP_*/
