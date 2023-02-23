package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.subsystems.LedMap;
import frc.robot.util.FireLeds;

public class Led extends SmartSubsystemBase {

    LedMap map;
    AddressableLED led;
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    AddressableLEDBuffer ledBuffer;

    public Led(LedMap map) {
        led = map.led;
        ledBuffer = map.ledBuffer;

        this.map = map;
        // PWM port 9
        // Must be a PWM header, not MXP or DIO

        led.setLength(ledBuffer.getLength());
        led.start();

    }

    public void setColor(int r, int g, int b) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            ledBuffer.setRGB(i, r, g, b);
        }
        led.setData(ledBuffer);
    }

    // } public CommandBase colorAlliance() {
    // return cmd("Set to Alliance Color").onExecute(() -> {
    // Alliance alliance = DriverStation.getAlliance();
    // if (alliance == Alliance.Blue) {
    // setColor(2, 15, 250);
    // Logger.getInstance().recordOutput("IndicateLEDs", "Blue");
    // } else {
    // setColor(250, 2, 2);
    // Logger.getInstance().recordOutput("IndicateLEDs", "Red");
    // }
    // });

    public CommandBase resetColor() {
        return runOnce(() -> {
            setColor(201, 198, 204);
            Logger.getInstance().recordOutput("IndicateLEDs", "White");

        });
    }

    public CommandBase setYellow() {
        return runOnce(() -> {
            setColor(222, 218, 11);
            Logger.getInstance().recordOutput("IndicateLEDs", "Yellow");

        });
    }

    public CommandBase setPurple() {
        return runOnce(() -> {
            setColor(133, 7, 168);
            Logger.getInstance().recordOutput("IndicateLEDs", "Purple");
        });
    }

    public CommandBase Fire() {
        return cmd("Make leds fire").onExecute(() -> {
            for (int i = 0; i < FireLeds.heat.length; i++) {
                Color color = new Color(FireLeds.heat[i] / 255.0, 0, 0);
                ledBuffer.setLED(i, color);
            }
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