package frc.robot.maps.subsystems;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

public class IntakeData implements LoggableInputs {

    public double setPoint;

    public static class Map implements LoggableMap<IntakeData> {
        public SmartMotorController motor;

        public Map() {
            this.motor = new SmartMotorController();
        }

        public Map(SmartMotorController motor) {
            this.motor = motor;

        }

        @Override
        public void updateData(IntakeData data) {
            motor.set(data.setPoint);

        }

    }

    @Override
    public void toLog(LogTable table) {
        table.put("MotorSetPoint", setPoint);

    }

    @Override
    public void fromLog(LogTable table) {
        setPoint = table.getDouble("MotorSetPoint", setPoint);
    }
}
