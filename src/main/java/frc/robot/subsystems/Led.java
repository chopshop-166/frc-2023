package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public LedBehavior[] ledBehaviors = { LedBehavior.None, LedBehavior.None, LedBehavior.ColdFire };
    private int spinCounter;
    private int ledPosition = 1;
    private boolean isFlashing;
    private int rainbowFirstPixelHue = 0;

    private final Timer flashTimer = new Timer();

    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
    BooleanSubscriber groundSub = ntinst.getBooleanTopic("Arm/Below Ground").subscribe(false);
    BooleanSubscriber autoBalanceState = ntinst.getBooleanTopic("Auto/Balance").subscribe(false);

    enum LedSection {
        Top(1),
        Bottom(0),
        All(2);

        private int section;

        private LedSection(int section) {

            this.section = section;
        }

        public int getSection() {
            return section;
        }
    }

    enum LedBehavior {
        Yellow,
        Purple,
        ColdFire,
        ColorAlliance,
        GrabbedPiece,
        IntakeSpinning,
        BalanceLeds,
        StarPower,
        None;
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

        flashTimer.start();
    }

    public void setColor(int r, int g, int b, LedSection section) {
        int ledBufferStartValue = 1;
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
    }

    public void handleLeds(LedSection section) {

        switch (ledBehaviors[section.getSection()]) {
            case Yellow:
                setColor(255, 119, 0, section);
                Logger.getInstance().recordOutput("IndicateLEDs", "Yellow");
                break;
            case Purple:
                setColor(133, 7, 168, section);
                Logger.getInstance().recordOutput("IndicateLEDs", "Purple");
                break;
            case ColdFire:
                calculateColdFire(heat.length, 25);
                for (int i = 1; i < heat.length; i++) {
                    Color color = heatToColor(heat[i]);
                    ledBuffer.setLED(i, color);
                    ledBuffer.setLED(ledBuffer.getLength() - i - 1, color);
                }
                led.setData(ledBuffer);
                break;
            case ColorAlliance:
                Alliance alliance = DriverStation.getAlliance();
                if (alliance == Alliance.Blue) {
                    setColor(2, 15, 250, section);
                    Logger.getInstance().recordOutput("IndicateLEDs", "Blue");
                } else {
                    setColor(250, 2, 2, section);
                    Logger.getInstance().recordOutput("IndicateLEDs", "Red");
                }
                break;
            case GrabbedPiece:
                setColor(0, 255, 0, section);
                Logger.getInstance().recordOutput("IndicateLEDs", "Green");
                break;
            case IntakeSpinning:
                spinCounter++;
                if (spinCounter % 3 == 0) {
                    ledPosition++;
                }
                if (ledPosition == ledBuffer.getLength() / 4) {
                    ledPosition = 1;
                }
                setColor(0, 0, 0, section);
                ledBuffer.setRGB(ledPosition, 0, 255, 0);
                ledBuffer.setRGB(ledBuffer.getLength() - ledPosition - 1, 0, 255, 0);
                Logger.getInstance().recordOutput("IndicateLEDs", "Spinning");
                break;
            case BalanceLeds:
                if (autoBalanceState.get(false)) {
                    setColor(0, 255, 0, section);
                    Logger.getInstance().recordOutput("IndicateLEDs", "Green");
                } else {
                    if (flashTimer.advanceIfElapsed(.5)) {
                        isFlashing = !isFlashing;
                    }
                    if (isFlashing) {
                        setColor(255, 0, 0, section);
                        Logger.getInstance().recordOutput("IndicateLEDs", "Red");
                    } else {
                        setColor(0, 0, 0, section);
                        Logger.getInstance().recordOutput("IndicateLEDs", "Off");
                    }
                }
                break;
            case StarPower:
                for (var i = 0; i < ledBuffer.getLength(); i++) {
                    final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
                    ledBuffer.setHSV(i, (int) hue, 255, 128);

                }
                rainbowFirstPixelHue += 3;
                rainbowFirstPixelHue %= 180;
            case None:
                break;

        }

    }

    public CommandBase colorAlliance() {
        return runOnce(() -> {
            ledBehaviors[LedSection.Bottom.getSection()] = LedBehavior.ColorAlliance;
            ledBehaviors[LedSection.All.getSection()] = LedBehavior.None;
        }).ignoringDisable(true);
    }

    public CommandBase resetColor() {
        return runOnce(() -> {
            setColor(201, 198, 204, LedSection.All);
            Logger.getInstance().recordOutput("IndicateLEDs", "White");

        });
    }

    public CommandBase intakeSpinning() {
        return runOnce(() -> {
            ledBehaviors[LedSection.Bottom.getSection()] = LedBehavior.IntakeSpinning;
            ledBehaviors[LedSection.All.getSection()] = LedBehavior.None;
        });
    }

    public CommandBase setYellow() {
        return runOnce(() -> {
            ledBehaviors[LedSection.Top.getSection()] = LedBehavior.Yellow;
            ledBehaviors[LedSection.All.getSection()] = LedBehavior.None;
        });
    }

    public CommandBase setPurple() {
        return runOnce(() -> {
            ledBehaviors[LedSection.Top.getSection()] = LedBehavior.Purple;
            ledBehaviors[LedSection.All.getSection()] = LedBehavior.None;
        });
    }

    public CommandBase grabbedPiece() {
        return runOnce(() -> {
            ledBehaviors[LedSection.Bottom.getSection()] = LedBehavior.GrabbedPiece;
            ledBehaviors[LedSection.All.getSection()] = LedBehavior.None;
        });
    }

    public CommandBase balancing() {
        return runOnce(() -> {
            ledBehaviors[LedSection.Bottom.getSection()] = LedBehavior.BalanceLeds;
            ledBehaviors[LedSection.All.getSection()] = LedBehavior.None;
        });
    }

    public CommandBase bottomOff() {
        return runOnce(() -> {
            ledBehaviors[LedSection.Bottom.getSection()] = LedBehavior.None;
            setColor(0, 0, 0, LedSection.Bottom);
        });
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
        for (int i = 1; i < heat.length; i++) {
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

    public CommandBase ColdFire() {
        return cmd("Make leds cold fire").onInitialize(() -> {
            ledBehaviors[LedSection.Top.getSection()] = LedBehavior.None;
            ledBehaviors[LedSection.Bottom.getSection()] = LedBehavior.None;
            ledBehaviors[LedSection.All.getSection()] = LedBehavior.ColdFire;
        }).runsWhenDisabled(true);
    }

    public CommandBase starPower() {
        return runOnce(() -> {
            ledBehaviors[LedSection.All.getSection()] = LedBehavior.StarPower;
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

        boolean seesTag = SmartDashboard.getBoolean("Saw Tag", false);
        Color ledColor = (seesTag) ? Color.kGreen : Color.kRed;
        if (groundSub.get()) {
            setColor(255, 0, 0, LedSection.Top);
        }
        ledBuffer.setLED(0, ledColor);

        handleLeds(LedSection.Bottom);
        handleLeds(LedSection.Top);
        handleLeds(LedSection.All);

        led.setData(ledBuffer);
    }
}
