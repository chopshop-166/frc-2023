package frc.robot.subsystems;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.maps.subsystems.LedMap;

public class Led extends SmartSubsystemBase {

    private LedMap map;
    AddressableLED led = new AddressableLED(9);
   AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(60);
    public Led(LedMap map) {
        this.map = map;
        // PWM port 9
        // Must be a PWM header, not MXP or DIO
       

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        
        led.setLength(ledBuffer.getLength());

        // Set the data
        led.setData(ledBuffer);
        led.start();
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            ledBuffer.setRGB(i, 252, 250, 250);
        }

    }

    public CommandBase resetColor() {
        return runOnce(() -> {
            led.setData(ledBuffer);
            led.start();
            for (var i = 0; i < ledBuffer.getLength(); i++) {
                // Sets the specified LED to the RGB values for red
                ledBuffer.setRGB(i, 252, 250, 250);
            }
            System.out.println(ledBuffer.getLED(0));

        });
    }

    public CommandBase setYellow() {
        return runOnce(() -> {
            led.setData(ledBuffer);
            led.start();
            for (var i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, 247, 237, 35);

            }
            System.out.println(ledBuffer.getLED(0));

        });
    }

    public CommandBase setPurple() {
        return runOnce(() -> {
            led.setData(ledBuffer);
            led.start();
            for (var i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, 138, 71, 245);
            }
            System.out.println(ledBuffer.getLED(0));
        });
    }

    public CommandBase printColor() {
        return runOnce(() -> {
            System.out.println(ledBuffer.getLED(0));
        });
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
    }
}