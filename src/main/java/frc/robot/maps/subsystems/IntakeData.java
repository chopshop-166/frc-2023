package frc.robot.maps.subsystems;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.pneumatics.IDSolenoid;
import com.chopshop166.chopshoplib.pneumatics.MockDSolenoid;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.util.Color;

public class IntakeData implements LoggableInputs {

    public double motorSetPoint;
    public Value solenoidSetPoint = Value.kOff;
    public int gamePieceDistance;
    public int maxGamePieceDistance;
    public int minGamePieceDistance;
    public double[] currentAmps;

    public static class Map implements LoggableMap<IntakeData> {
        public SmartMotorController motor;
        public IDSolenoid solenoid;

        public Map() {
            this(new SmartMotorController(), new MockDSolenoid());
        }

        public Map(SmartMotorController motor, IDSolenoid solenoid) {
            this.motor = motor;
            this.solenoid = solenoid;
        }

        @Override
        public void updateData(IntakeData data) {
            motor.set(data.motorSetPoint);
            solenoid.set(data.solenoidSetPoint);
            data.currentAmps = motor.getCurrentAmps();
        }

    }

    // Logs the values of the variables
    @Override
    public void toLog(LogTable table) {
        table.put("MotorSetPoint", motorSetPoint);
        table.put("SolenoidSetPoint", solenoidSetPoint.toString());
        table.put("GamePieceDistance", gamePieceDistance);
        table.put("MotorCurrentAmps", currentAmps);

    }

    // Retrieves values of the variables
    @Override
    public void fromLog(LogTable table) {
        double[] colorDoubleArray = { 0, 0, 0 };

        motorSetPoint = table.get("MotorSetPoint", motorSetPoint);
        solenoidSetPoint = Value.valueOf(table.get("SolenoidSetPoint", solenoidSetPoint.toString()));
        gamePieceDistance = table.get("GamePieceDistance", gamePieceDistance);
        this.currentAmps = table.get("MotorCurrentAmps", currentAmps);
    }
}
