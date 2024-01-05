package frc.robot.subsystems.leds;

import com.chopshop166.chopshoplib.leds.patterns.IndicatorPattern;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class TagPattern extends IndicatorPattern {

    public TagPattern() {
        super(Color.kGreen, Color.kRed, () -> SmartDashboard.getBoolean("Saw Tag", false));
    }

    @Override
    public String toString() {
        return "TagPattern()";
    }
}
