
#ifndef AREABASEDCELLCYCLEMODEL_HPP_
#define AREABASEDCELLCYCLEMODEL_HPP_

#include "AbstractCellCycleModel.hpp"
#include "RandomNumberGenerator.hpp"

/*
 * \todo Document this class
 */
class AreaBasedCellCycleModel : public AbstractCellCycleModel
{
private:

    /**
     * A reference target area.
     * Initialised to 1.0 in the constructor.
     */
    double mReferenceTargetArea;

    /**
     * A target area.
     * Initialised to 1.0 in the constructor.
     */
    double mTargetArea;

    /**
     * The maximum growth rate of the target area.
     * Initialised to 1.0 in the constructor.
     */
    double mMaxGrowthRate;

    /** Needed for serialization. */
    friend class boost::serialization::access;
    /**
     * Archive the cell-cycle model and random number generator, never used directly - boost uses this.
     *
     * @param archive the archive
     * @param version the current version of this class
     */
    template<class Archive>
    void serialize(Archive & archive, const unsigned int version)
    {
        archive & boost::serialization::base_object<AbstractCellCycleModel>(*this);
        archive & mReferenceTargetArea;
        archive & mTargetArea;
        archive & mMaxGrowthRate;
    }

protected:

    /**
     * Protected copy-constructor for use by CreateCellCycleModel().
     * The only way for external code to create a copy of a cell cycle model
     * is by calling that method, to ensure that a model of the correct subclass is created.
     * This copy-constructor helps subclasses to ensure that all member variables are correctly copied when this happens.
     *
     * This method is called by child classes to set member variables for a daughter cell upon cell division.
     * Note that the parent cell cycle model will have had ResetForDivision() called just before CreateCellCycleModel() is called,
     * so performing an exact copy of the parent is suitable behaviour. Any daughter-cell-specific initialisation
     * can be done in InitialiseDaughterCell().
     *
     * @param rModel the cell cycle model to copy.
     */
    AreaBasedCellCycleModel(const AreaBasedCellCycleModel& rModel);

public:

    /**
     * Constructor.
     */
    AreaBasedCellCycleModel();

    /**
     * Overridden ReadyToDivide() method.
     *
     * @return whether the cell is ready to divide.
     */
    virtual bool ReadyToDivide();

    /**
     * Overridden builder method to create new copies of this cell-cycle model.
     *
     * @return a pointer to the AreaBasedCellCycleModel created.
     */
    AbstractCellCycleModel* CreateCellCycleModel();

    /**
     * Overridden ResetForDivision() method.
     */
    void ResetForDivision();

    /**
     * Overridden GetAverageTransitCellCycleTime() method.
     *
     * @return the average cell cycle time for cells of transit proliferative type
     */
    double GetAverageTransitCellCycleTime();

    /**
     * Overridden GetAverageStemCellCycleTime() method.
     *
     * @return the average cell cycle time for cells of stem proliferative type
     */
    double GetAverageStemCellCycleTime();

    /**
     * Get the reference target area.
     *
     * @return the reference target area.
     */
    double GetReferenceTargetArea();

    /**
     * Get the target area.
     *
     * @return the target area.
     */
    double GetTargetArea();

    /**
     * Set the reference target area. The default value is 1.0.
     *
     * @param referenceTargetArea the new value of #mReferenceTargetArea
     */
    void SetReferenceTargetArea(double referenceTargetArea);

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
     * Overridden OutputCellCycleModelParameters() method.
     *
     * @param rParamsFile the file stream to which the parameters are output
     */
    virtual void OutputCellCycleModelParameters(out_stream& rParamsFile);
};

#include "SerializationExportWrapper.hpp"
// Declare identifier for the serializer
CHASTE_CLASS_EXPORT(AreaBasedCellCycleModel)

#endif /* AREABASEDCELLCYCLEMODEL_HPP_ */
