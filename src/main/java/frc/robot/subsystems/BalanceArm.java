package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.pneumatics.IDSolenoid;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.subsystems.BalanceArmMap;

public class BalanceArm extends SmartSubsystemBase {

    private BalanceArmMap map;
    private IDSolenoid solenoid;
    private boolean state;

    public BalanceArm(BalanceArmMap map) {
        this.map = map;
        this.solenoid = map.getSolenoid();
    }

    public CommandBase pushStation(boolean down) {
        return runOnce(() -> {
            solenoid.set(down ? Value.kForward : Value.kReverse);
            this.state = down;
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
        Logger.getInstance().recordOutput("BalanceArm/pushingDown", false);
    }
}