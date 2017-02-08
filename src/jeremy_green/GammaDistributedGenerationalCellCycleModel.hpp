
#ifndef GAMMADISTRIBUTEDGENERATIONALCELLCYCLEMODEL_HPP_
#define GAMMADISTRIBUTEDGENERATIONALCELLCYCLEMODEL_HPP_

#include "AbstractSimpleCellCycleModel.hpp"
#include "RandomNumberGenerator.hpp"

/**
 * A stochastic cell-cycle model where cells divide for a specified number of generations,
 * with each cell-cycle time drawn from a gamma distribution with specified shape and scale
 * parameters. Note that distinct cell-cycle phases (G1, S, G2, M) are NOT included in this
 * model.
 */
class GammaDistributedGenerationalCellCycleModel : public AbstractSimpleCellCycleModel
{
private:

    /** The shape parameter of the gamma distribution. This must be a positive real number. */
    double mShape;

    /** The scale parameter of the gamma distribution. This must be a positive real number. */
    double mScale;

    /** The generation of this cell. */
    unsigned mGeneration;

    /** How many generations a transit cell lives for before becoming fully differentiated. */
    unsigned mMaxGeneration;

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
        archive & boost::serialization::base_object<AbstractSimpleCellCycleModel>(*this);

        // Make sure the RandomNumberGenerator singleton gets saved too
        SerializableSingleton<RandomNumberGenerator>* p_wrapper = RandomNumberGenerator::Instance()->GetSerializationWrapper();
        archive & p_wrapper;

        archive & mShape;
        archive & mScale;
        archive & mGeneration;
        archive & mMaxGeneration;
    }

protected:

    /**
     * Protected copy-constructor for use by CreateCellCycleModel.
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
    GammaDistributedGenerationalCellCycleModel(const GammaDistributedGenerationalCellCycleModel& rModel);

public:

    /**
     * Constructor.
     */
    GammaDistributedGenerationalCellCycleModel();

    /**
     * Overridden SetCellCycleDuration() method.
     */
    double GetAverageTransitCellCycleTime();

    /**
     * Overridden SetCellCycleDuration() method.
     */
    double GetAverageStemCellCycleTime();

    /**
     * Overridden SetCellCycleDuration() method.
     */
    void SetCellCycleDuration();

    /**
     * Overridden ResetForDivision() method.
     */
    void ResetForDivision();

    /**
     * Overridden InitialiseDaughterCell() method.
     */
    void InitialiseDaughterCell();

    /**
     * Sets the cell's generation.
     *
     * @param generation the cell's generation
     */
    void SetGeneration(unsigned generation);

    /**
     * @return the cell's generation.
     */
    unsigned GetGeneration() const;

    /**
     * Set mMaxGeneration.
     *
     * @param maxGeneration the new value of mMaxGeneration
     */
    void SetMaxGeneration(unsigned maxGeneration);

    /**
     * @return mMaxGeneration
     */
    unsigned GetMaxGeneration() const;

    /**
     * Overridden builder method to create new copies of this cell-cycle model.
     *
     * @return a pointer to the GammaDistributedGenerationalCellCycleModel created.
     */
    AbstractCellCycleModel* CreateCellCycleModel();

    /**
     * Set mShape.
     *
     * @param shape the value of the shape parameter
     */
    void SetShape(double shape);

    /**
     * @return mScale.
     *
     * @param scale the value of the scale parameter
     */
    void SetScale(double scale);

    /**
     * @return mShape.
     */
    double GetShape() const;

    /**
     * @return mScale.
     */
    double GetScale() const;

    /**
     * Overridden OutputCellCycleModelParameters() method.
     *
     * @param rParamsFile the file stream to which the parameters are output
     */
    virtual void OutputCellCycleModelParameters(out_stream& rParamsFile);
};

#include "SerializationExportWrapper.hpp"
// Declare identifier for the serializer
CHASTE_CLASS_EXPORT(GammaDistributedGenerationalCellCycleModel)

#endif /* GAMMADISTRIBUTEDGENERATIONALCELLCYCLEMODEL_HPP_ */
