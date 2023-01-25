package com.chopshop166.chopshoplib.logging;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

/**
 * A subsystem that supports logging via inheritance.
 * 
 * @param <D> The data object for input/output.
 * @param <M> The hardware map.
 */
public abstract class LoggedSubsystem<D extends LoggableInputs, M extends LoggableMap<D>>
        extends SmartSubsystemBase {

    /** The input/output data. */
    private final D ioData;
    /** The hardware map. */
    private final M map;

    /**
     * Constructor.
     * 
     * @param ioData The data object to use for i/o.
     * @param map The hardware mapping.
     */
    public LoggedSubsystem(final D ioData, final M map) {
        super();
        this.ioData = ioData;
        this.map = map;
    }

    /**
     * Get the data for i/o.
     * 
     * @return The data object.
     */
    public D getData() {
        return this.ioData;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Use this for any background processing
        this.map.updateData(this.ioData);
        Logger.getInstance().processInputs(this.getName(), this.ioData);
    }

}
