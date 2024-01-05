package frc.robot.subsystems.leds;

import com.chopshop166.chopshoplib.leds.Pattern;
import com.chopshop166.chopshoplib.leds.SegmentBuffer;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class BalancePattern extends Pattern {
    private Timer timer = new Timer();
    private boolean isFlashing;

    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private BooleanSubscriber autoBalanceState = inst.getBooleanTopic("Auto/Balance").subscribe(false);

    @Override
    public void initialize(SegmentBuffer buffer) {
        isFlashing = false;
    }

    @Override
    public void update(SegmentBuffer buffer) {
        if (autoBalanceState.get(false)) {
            buffer.setAll(Color.kGreen);
        } else {
            if (timer.advanceIfElapsed(.5)) {
                isFlashing = !isFlashing;
            }
            buffer.setAll(isFlashing ? Color.kRed : Color.kBlack);
        }
    }

    @Override
    public String toString() {
        return "BalancePattern()";
    }
}
