package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.pneumatics.IDSolenoid;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.BalanceArmMap;
import frc.robot.maps.subsystems.BalanceArmMap.Data;

public class BalanceArm extends SmartSubsystemBase {

    private BalanceArmMap map;
    Data data = new Data();

    public BalanceArm(BalanceArmMap map) {
        this.map = map;
    }

    public Command pushDown() {
        return pushStation(Value.kForward);
    }

    public Command pushUp() {
        return pushStation(Value.kReverse);
    }

    private Command pushStation(Value val) {
        return runOnce(() -> {
            data.solenoidSetPoint = val;
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
        this.map.updateData(data);
    }
}