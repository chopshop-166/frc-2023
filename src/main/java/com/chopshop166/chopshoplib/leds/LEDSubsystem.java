package com.chopshop166.chopshoplib.leds;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;

public class LEDSubsystem extends SmartSubsystemBase {

    protected LedMap map;
    protected AddressableLED led;
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    protected LEDStripBuffer ledBuffer;

    public LEDSubsystem(LedMap map) {
        this.led = map.led;
        this.ledBuffer = map.ledBuffer;
        this.map = map;

        led.setLength(ledBuffer.getLength());
        led.start();
    }

    /**
     * Generic "set a tag to a pattern" command
     * 
     * @param tag     The tag to set.
     * @param pattern The pattern to set it to.
     * @return A command object.
     */
    public Command setPattern(String tag, Pattern pattern) {
        return runOnce(() -> {
            ledBuffer.setPattern(tag, pattern);
        }).ignoringDisable(true);
    }

    /**
     * Generic "set a tag to a pattern" command
     * 
     * @param tag       The tag to set.
     * @param pattern   The pattern to set it to.
     * @param indicator The name to record in the log.
     * @return A command object.
     */
    public Command setPattern(String tag, Pattern pattern, String indicator) {
        return setPattern(tag, pattern).andThen(logIndicator(indicator));
    }

    /**
     * Generic "set everything to a pattern" command
     * 
     * @param pattern The pattern to set it to.
     * @return A command object.
     */
    public Command setGlobalPattern(Pattern pattern) {
        return runOnce(() -> {
            ledBuffer.setGlobalPattern(pattern);
        }).ignoringDisable(true);
    }

    /**
     * Log a message saying what the indicator lights are doing.
     * 
     * @param name The name of the pattern.
     * @return A command object.
     */
    public Command logIndicator(String name) {
        return runOnce(() -> {
            Logger.recordOutput("IndicateLEDs", name);
        }).ignoringDisable(true);
    }

    @Override
    public void reset() {
        // Nothing to reset here
    }

    @Override
    public void safeState() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Use this for any background processing
        ledBuffer.update(led);
    }
}
