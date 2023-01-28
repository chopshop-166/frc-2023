package frc.robot.maps.subsystems;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.pneumatics.IDSolenoid;
import com.chopshop166.chopshoplib.pneumatics.MockDSolenoid;
import com.chopshop166.chopshoplib.sensors.IColorSensor;
import com.chopshop166.chopshoplib.sensors.MockColorSensor;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.util.Color;

public class IntakeData implements LoggableInputs {

    public double motorSetPoint;
    public Value solenoidSetPoint;
    public double[] detectedColor;

    public static class Map implements LoggableMap<IntakeData> {
        public SmartMotorController motor;
        public IDSolenoid solenoid;
        public IColorSensor colorSensor;

        public Map() {
            this.motor = new SmartMotorController();
            this.solenoid = new MockDSolenoid();
            this.colorSensor = new MockColorSensor();
        }

        public Map(SmartMotorController motor) {
            this.motor = motor;
        }

        public Map(IDSolenoid solenoid) {
            this.solenoid = solenoid;
        }

        public Map(IColorSensor colorSensor) {
            this.colorSensor = colorSensor;
        }

        @Override
        public void updateData(IntakeData data) {
            motor.set(data.motorSetPoint);
            solenoid.set(data.solenoidSetPoint);
            Color sensorColor = colorSensor.getColor();
            data.detectedColor = new double[] { sensorColor.red, sensorColor.green, sensorColor.blue };
        }

    }

    @Override
    public void toLog(LogTable table) {
        table.put("MotorSetPoint", motorSetPoint);
        table.put("SolenoidSetPoint", solenoidSetPoint.toString());
        table.put("DetetectedColor", detectedColor);
    }

    @Override
    public void fromLog(LogTable table) {
        motorSetPoint = table.getDouble("MotorSetPoint", motorSetPoint);
        solenoidSetPoint = Value.valueOf(table.getString("SolenoidSetPoint", solenoidSetPoint.toString()));
        detectedColor = table.getDoubleArray("DetectedColor", detectedColor);
    }
}
