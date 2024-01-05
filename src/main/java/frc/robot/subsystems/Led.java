package frc.robot.subsystems;

import com.chopshop166.chopshoplib.leds.LEDSubsystem;
import com.chopshop166.chopshoplib.leds.LedMap;
import com.chopshop166.chopshoplib.leds.patterns.AlliancePattern;
import com.chopshop166.chopshoplib.leds.patterns.ColdFirePattern;
import com.chopshop166.chopshoplib.leds.patterns.RainbowRoad;
import com.chopshop166.chopshoplib.leds.patterns.SolidColorPattern;
import com.chopshop166.chopshoplib.leds.patterns.SpinPattern;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.BalancePattern;
import frc.robot.subsystems.leds.GroundSubPattern;
import frc.robot.subsystems.leds.TagPattern;

public class Led extends LEDSubsystem {

    public Led(LedMap map) {
        super(map);

        ledBuffer.setPattern("Arm Status", new GroundSubPattern());
        ledBuffer.setPattern("Sees Tag", new TagPattern());
        // This one is length / 2 because the buffer has a mirrored other half
        ledBuffer.setPattern("Fire", new ColdFirePattern(ledBuffer.getLength() / 2));
    }

    public Command colorAlliance() {
        return setPattern("Alliance", new AlliancePattern(), "Alliance");
    }

    public Command resetColor() {
        return setGlobalPattern(new SolidColorPattern(new Color(201, 198, 204)));
    }

    public Command intakeSpinning() {
        return setPattern("Intake", new SpinPattern(), "Spinning");
    }

    public Command setYellow() {
        return setPattern("HP Signal", new SolidColorPattern(new Color(255, 119, 0)), "Yellow");
    }

    public Command setOrange() {
        return setPattern("Intake", new SolidColorPattern(new Color(255, 84, 174)), "Orange");
    }

    public Command setPurple() {
        return setPattern("HP Signal", new SolidColorPattern(new Color(133, 7, 168)), "Purple");
    }

    public Command grabbedPiece() {
        return setPattern("Balance Status", new SolidColorPattern(Color.kGreen), "Green");
    }

    public Command balancing() {
        return setPattern("Balance Status", new BalancePattern(), "Balancing");
    }

    public Command bottomOff() {
        return setPattern("Bottom", new SolidColorPattern(Color.kBlack), "Off");
    }

    public Command ColdFire() {
        return setPattern("Fire", new ColdFirePattern(ledBuffer.getLength() / 2), "Cold Fire")
                .withName("Make LEDs cold fire");
    }

    public Command starPower() {
        return setPattern("Rainbow", new RainbowRoad(), "Star Power");
    }
}
