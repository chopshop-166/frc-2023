package frc.robot.subsystems.leds;

import com.chopshop166.chopshoplib.leds.patterns.IndicatorPattern;

import edu.wpi.first.wpilibj.util.Color;

public class GroundSubPattern extends IndicatorPattern {

    public GroundSubPattern() {
        super(Color.kRed, Color.kGreen, "Arm/Below Ground");
    }

    @Override
    public String toString() {
        return "GroundSubPattern()";
    }

}
