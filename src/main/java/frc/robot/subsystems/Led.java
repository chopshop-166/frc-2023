package frc.robot.subsystems;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.subsystems.LedMap;

public class Led extends SmartSubsystemBase {

    private LedMap map;
    private Led defaultColor;
    private int R;
    private int G;
    private int B;
    AddressableLED led = new AddressableLED(0);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(60);

    public Led(LedMap map) {

        this.map = map;
        // PWM port 9
        // Must be a PWM header, not MXP or DIO

        led.setLength(ledBuffer.getLength());

        // Set the data
        led.setData(ledBuffer);
        led.start();
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            ledBuffer.setRGB(i, 252, 250, 250);
        }

    }

    public void setColor(int R, int G, int B) {
        led.setData(ledBuffer);
        led.start();
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            ledBuffer.setRGB(i, R, G, B);
        }
    }

    public CommandBase colorAlliance() {
        return cmd("Set to Alliance Color").onExecute(() -> {
            Alliance alliance = DriverStation.getAlliance();
            if (alliance == Alliance.Blue) {
                setColor(2, 15, 250);
            } else {
                setColor(250, 2, 2);
            }
        });
    }

    public CommandBase resetColor() {
        return runOnce(() -> {
            setColor(201, 198, 204);
            System.out.println(ledBuffer.getLED(0));

        });
    }

    public CommandBase setYellow() {
        return runOnce(() -> {
            setColor(222, 218, 11);
            System.out.println(ledBuffer.getLED(0));

        });
    }

    public CommandBase setPurple() {
        return runOnce(() -> {
            setColor(133, 7, 168);
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