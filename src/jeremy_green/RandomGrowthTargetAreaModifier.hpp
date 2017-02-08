
#ifndef RANDOMGROWTHTARGETAREAMODIFIER_HPP_
#define RANDOMGROWTHTARGETAREAMODIFIER_HPP_

#include "ChasteSerialization.hpp"
#include <boost/serialization/base_object.hpp>

#include "AbstractTargetAreaModifier.hpp"
#include "VertexBasedCellPopulation.hpp"

/**
 * \todo Document this class
 */
template<unsigned DIM>
class RandomGrowthTargetAreaModifier : public AbstractTargetAreaModifier<DIM>
{
private:

    /**
     * The maximum growth rate of the target area.
     * Initialised to a default value of zero in the constructor.
     */
    double mMaxGrowthRate;

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
        archive & mMaxGrowthRate;
    }

public:

    /**
     * Default constructor.
     */
    RandomGrowthTargetAreaModifier();

    /**
     * Destructor.
     */
    virtual ~RandomGrowthTargetAreaModifier();

    /**
     * Set mMaxGrowthRate.
     *
     * @param maxGrowthRate the new value of mMaxGrowthRate
     */
    void SetMaxGrowthRate(double maxGrowthRate);

    /**
     * @return mMaxGrowthRate
     */
    double GetMaxGrowthRate();

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
EXPORT_TEMPLATE_CLASS_SAME_DIMS(RandomGrowthTargetAreaModifier)

#endif /*RANDOMGROWTHTARGETAREAMODIFIER_HPP_*/
