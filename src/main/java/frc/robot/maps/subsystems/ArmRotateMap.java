package frc.robot.maps.subsystems;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.math.controller.PIDController;

public class ArmRotateMap {
    final public SmartMotorController motor;
    final public PIDController pid;
    public final double topAngle;
    public final double bottomAngle;

    public ArmRotateMap() {
        this.motor = new SmartMotorController();
        this.topAngle = 85;
        this.bottomAngle = 5;
        this.pid = new PIDController(0.01, 0, 0);
    }

    public ArmRotateMap(SmartMotorController motor, double topAngle, double bottomAngle, PIDController pid) {
        this.motor = motor;
        this.topAngle = topAngle;
        this.bottomAngle = bottomAngle;
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
