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

public class Led extends SmartSubsystemBase {

    LedMap map;
    AddressableLED led;
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    AddressableLEDBuffer ledBuffer;
    public final byte[] heat;

    enum LedSection {
        Top,
        Bottom,
        All
    }

    public Led(LedMap map) {
        led = map.led;
        ledBuffer = map.ledBuffer;

        this.map = map;
        // PWM port 9
        // Must be a PWM header, not MXP or DIO

        led.setLength(ledBuffer.getLength());
        led.start();

        heat = new byte[ledBuffer.getLength() / 2];

    }

    public void setColor(int r, int g, int b, LedSection section) {
        int ledBufferStartValue = 0;
        int ledBufferEndValue = (ledBuffer.getLength() * 3) / 4;

        if (section == LedSection.Top) {
            ledBufferStartValue = ledBuffer.getLength() / 4;
            ledBufferEndValue = ledBuffer.getLength() / 2;
        } else if (section == LedSection.Bottom) {
            ledBufferEndValue = ledBuffer.getLength() / 4;
        }

        for (var i = ledBufferStartValue; i < ledBufferEndValue; i++) {
            ledBuffer.setRGB(i, r, g, b);
            ledBuffer.setRGB(ledBuffer.getLength() - i - 1, r, g, b);
        }
        led.setData(ledBuffer);
    }

    public CommandBase colorAlliance() {
        return cmd("Set to Alliance Color").onExecute(() -> {
            Alliance alliance = DriverStation.getAlliance();
            if (alliance == Alliance.Blue) {
                setColor(2, 15, 250, LedSection.Top);
                Logger.getInstance().recordOutput("IndicateLEDs", "Blue");
            } else {
                setColor(250, 2, 2, LedSection.Top);
                Logger.getInstance().recordOutput("IndicateLEDs", "Red");
            }
        }).runsWhenDisabled(true);
    }

    public CommandBase resetColor() {
        return runOnce(() -> {
            setColor(201, 198, 204, LedSection.All);
            Logger.getInstance().recordOutput("IndicateLEDs", "White");

        });
    }

    public CommandBase setYellow() {
        return runOnce(() -> {
            setColor(222, 218, 11, LedSection.Bottom);
            Logger.getInstance().recordOutput("IndicateLEDs", "Yellow");

        });
    }

    public CommandBase setPurple() {
        return runOnce(() -> {
            setColor(133, 7, 168, LedSection.Bottom);
            Logger.getInstance().recordOutput("IndicateLEDs", "Purple");
        });
    }

    public CommandBase coldFire() {
        return cmd("Make leds cold fire").onExecute(() -> {
            calculateColdFire(heat.length, 50);
            for (int i = 0; i < heat.length; i++) {
                Color color = heatToColor(heat[i]);
                ledBuffer.setLED(i, color);
                ledBuffer.setLED(ledBuffer.getLength() - i - 1, color);
            }
            led.setData(ledBuffer);
        }).runsWhenDisabled(true);
    }

    private Color heatToColor(byte h) {
        // Scale 'heat' down from 0-255 to 0-191
        byte t192 = (byte) Math.round((h / 255.0) * 191);

        // calculate ramp up from
        byte heatramp = (byte) (t192 & 0x3F); // 0..63
        heatramp <<= 2; // scale up to 0..252

        // figure out which third of the spectrum we're in:
        if (t192 > 0x80) { // hottest
            return new Color(heatramp, 255, 255);
        } else if (t192 > 0x40) { // middle
            return new Color(0, heatramp, 255);
        } else { // coolest
            return new Color(0, 0, heatramp);
        }
    }

    public void calculateColdFire(int flameHeight, int sparks) {
        // Cool down each cell a little
        for (int i = 0; i < heat.length; i++) {
            int cooldown = (int) (Math.random() * ((flameHeight * 10) / heat.length + 2));

            if (cooldown > heat[i]) {
                heat[i] = 0;
            } else {
                heat[i] = (byte) (heat[i] - cooldown);
            }
        }

        // Heat from each cell drifts up and diffuses slightly
        for (int k = heat.length - 1; k >= 2; k--) {
            heat[k] = (byte) ((heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3);
        }

        // Randomly ignite new sparks near bottom of the flame
        if (Math.random() * 255 < sparks) {
            int y = (int) (Math.random() * 7);
            heat[y] = (byte) (heat[y] + (int) (Math.random() * (160 - 255 + 1) + 160));
        }
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