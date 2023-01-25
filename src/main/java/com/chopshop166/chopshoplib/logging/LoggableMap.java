package com.chopshop166.chopshoplib.logging;

import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Interface for a subsystem map that allows logging.
 */
@FunctionalInterface
public interface LoggableMap<D extends LoggableInputs> {

    /**
     * Update i/o data to be sent.
     * 
     * @param data The data to update to/from.
     */
    void updateData(D data);

}
