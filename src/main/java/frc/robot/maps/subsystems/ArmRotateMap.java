package frc.robot.maps.subsystems;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.math.controller.PIDController;

public class ArmRotateMap {
    public final SmartMotorController motor;
    public final PIDController pid;
    public final double softMaxAngle;
    public final double softMinAngle;
    public double hardMaxAngle;
    public double hardMinAngle;

    public ArmRotateMap() {
        this.motor = new SmartMotorController();
        this.softMaxAngle = 85;
        this.softMinAngle = 5;
        this.pid = new PIDController(0.01, 0, 0);
    }

    public ArmRotateMap(SmartMotorController motor, double softMaxAngle, double softMinAngle, double hardMaxAngle,
            double hardMinAngle, PIDController pid) {
        this.motor = motor;
        this.softMaxAngle = softMaxAngle;
        this.softMinAngle = softMinAngle;
        this.hardMaxAngle = hardMaxAngle;
        this.hardMinAngle = hardMinAngle;
        this.pid = pid;
    }

    public void updateData(Data data) {
        motor.set(data.setPoint);
        data.degrees = motor.getEncoder().getDistance();
        data.velocityDegreesPerSecond = motor.getEncoder().getRate();
        data.currentAmps = motor.getCurrentAmps();
        data.tempCelcius = motor.getTemperatureC();
    }

    public static class Data implements LoggableInputs {
        public double setPoint;
        public double degrees;
        public double velocityDegreesPerSecond;
        public double[] currentAmps;
        public double[] tempCelcius;

        @Override
        public void toLog(LogTable table) {
            table.put("MotorSetpoint", setPoint);
            table.put("RotationDegrees", degrees);
            table.put("MotorVelocityMetersPerSeconds", velocityDegreesPerSecond);
            table.put("MotorTempCelcius", tempCelcius);
            table.put("MotorCurrentAmps", currentAmps);

        }

        @Override
        public void fromLog(LogTable table) {
            this.setPoint = table.getDouble("MotorSetpoint", setPoint);
            this.degrees = table.getDouble("RoationDegrees", degrees);
            this.velocityDegreesPerSecond = table.getDouble("MotorVelocityMetersPerSeconds", velocityDegreesPerSecond);
            this.currentAmps = table.getDoubleArray("MotorCurrentAmps", currentAmps);
            this.tempCelcius = table.getDoubleArray("MotorTempCelcius", tempCelcius);

        }
    }

}
