package frc.robot.maps.subsystems;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.chopshop166.chopshoplib.pneumatics.IDSolenoid;
import com.chopshop166.chopshoplib.pneumatics.MockDSolenoid;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class BalanceArmMap {

    private final IDSolenoid solenoid;

    public BalanceArmMap() {
        this(new MockDSolenoid());
    }

    public BalanceArmMap(IDSolenoid solenoid) {
        this.solenoid = solenoid;
    }

    public IDSolenoid getSolenoid() {
        return solenoid;
    }

    public void updateData(Data data) {
        solenoid.set(data.solenoidSetPoint);
    }

    public static class Data implements LoggableInputs {
        public Value solenoidSetPoint = Value.kOff;

        @Override
        public void toLog(LogTable table) {
            table.put("SolenoidSetPoint", solenoidSetPoint.toString());

        }

        @Override
        public void fromLog(LogTable table) {
            solenoidSetPoint = Value.valueOf(table.getString("SolenoidSetPoint", solenoidSetPoint.toString()));
        }
    }
}
