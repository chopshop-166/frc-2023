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
    public Value solenoidSetPoint = Value.kOff;
    public Color sensorColor;
    public int gamePieceDistance;
    public int maxGamePieceDistance;
    public int minGamePieceDistance;
    public double[] currentAmps;

    public static class Map implements LoggableMap<IntakeData> {
        public SmartMotorController motor;
        public IDSolenoid solenoid;
        public IColorSensor colorSensor;

        public Map() {
            this.motor = new SmartMotorController();
            this.solenoid = new MockDSolenoid();
            this.colorSensor = new MockColorSensor();
        }

        public Map(SmartMotorController motor, IDSolenoid solenoid, IColorSensor colorSensor) {
            this.motor = motor;
            this.solenoid = solenoid;
            this.colorSensor = colorSensor;
        }

        @Override
        public void updateData(IntakeData data) {
            motor.set(data.motorSetPoint);
            solenoid.set(data.solenoidSetPoint);
            data.sensorColor = colorSensor.getColor();
            data.gamePieceDistance = colorSensor.getProximity();
            data.currentAmps = motor.getCurrentAmps();
        }

    }

    // Logs the values of the variables
    @Override
    public void toLog(LogTable table) {
        table.put("MotorSetPoint", motorSetPoint);
        table.put("SolenoidSetPoint", solenoidSetPoint.toString());
        double[] detectedColor = new double[] { sensorColor.red, sensorColor.green, sensorColor.blue };
        table.put("DetetectedColor", detectedColor);
        table.put("GamePieceDistance", gamePieceDistance);
        table.put("MotorCurrentAmps", currentAmps);

    }

    // Retrieves values of the variables
    @Override
    public void fromLog(LogTable table) {
        double[] colorDoubleArray = { 0, 0, 0 };

        motorSetPoint = table.getDouble("MotorSetPoint", motorSetPoint);
        solenoidSetPoint = Value.valueOf(table.getString("SolenoidSetPoint", solenoidSetPoint.toString()));
        double[] detectedColor = table.getDoubleArray("DetectedColor", colorDoubleArray);
        sensorColor = new Color(detectedColor[0], detectedColor[1], detectedColor[2]);
        gamePieceDistance = (int) table.getInteger("GamePieceDistance", gamePieceDistance);
        this.currentAmps = table.getDoubleArray("MotorCurrentAmps", currentAmps);
    }
}
