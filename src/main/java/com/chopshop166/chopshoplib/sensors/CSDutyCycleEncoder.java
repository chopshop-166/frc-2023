package com.chopshop166.chopshoplib.sensors;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class CSDutyCycleEncoder extends DutyCycleEncoder implements IAbsolutePosition {

    /**
     * Create the Duty Cycle Encoder
     *
     * @param channel The channel the sensor is connected to.
     */
    public CSDutyCycleEncoder(int channel) {
        super(channel);
    }

    /**
     * Create the Duty Cycle Encoder
     *
     * @param dutyCycle The DutyCycle object.
     */
    public CSDutyCycleEncoder(DutyCycle dutyCycle) {
        super(dutyCycle);
    }

    /**
     * Create the Duty Cycle Encoder
     *
     * @param source The Source object.
     */
    public CSDutyCycleEncoder(DigitalSource source) {
        super(source);
    }

}
