package frc.robot.maps.subsystems;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.math.controller.PIDController;

public class ArmExtendMap {
    public PIDController pid;
    public SmartMotorController extendMotor;
    public double softMaxDistance;
    public double softMinDistance;
    public double hardMaxDistance;
    public double hardMinDistance;
    public double armStartLength;
    public double pivotHeight;

    public ArmExtendMap() {
        this.pid = new PIDController(0, 0, 0);
        this.extendMotor = new SmartMotorController();
        this.softMaxDistance = 20;
        this.softMinDistance = 1;
        this.pivotHeight = 46.654;
    }

    public ArmExtendMap(SmartMotorController motor, double softMaxDistance, double softMinDistance,
            double hardMaxDistance,
            double hardMinDistance, PIDController pid, double armPivotHeight, double armStartLength) {
        this.extendMotor = motor;
        this.softMaxDistance = softMaxDistance;
        this.softMinDistance = softMinDistance;
        this.hardMaxDistance = hardMaxDistance;
        this.hardMinDistance = hardMinDistance;
        this.pid = pid;
        this.pivotHeight = armPivotHeight;
        this.armStartLength = armStartLength;
    }

    public void updateData(Data data) {
        extendMotor.set(data.setPoint);
        data.distanceInches = extendMotor.getEncoder().getDistance();
        data.velocityInchesPerSec = extendMotor.getEncoder().getRate();
        data.currentAmps = extendMotor.getCurrentAmps();
        data.tempCelcius = extendMotor.getTemperatureC();
    }

    public static class Data implements LoggableInputs {
        public double setPoint;
        public double distanceInches;
        public double velocityInchesPerSec;
        public double[] currentAmps;
        public double[] tempCelcius;

        // Logs the values of the variables
        @Override
        public void toLog(LogTable table) {
            table.put("MotorSetpoint", setPoint);
            table.put("MotorDistanceInches", distanceInches);
            table.put("MotorVelocityInchesPerSec", velocityInchesPerSec);
            table.put("MotorCurrentAmps", currentAmps);
            table.put("MotorTempCelcius", tempCelcius);
        }

        // Retrieves the values of the variables
        @Override
        public void fromLog(LogTable table) {
            setPoint = table.getDouble("MotorSetpoint", setPoint);
            distanceInches = table.getDouble("MotorDistanceInches", distanceInches);
            velocityInchesPerSec = table.getDouble("MotorVelocityInchesPerSec", velocityInchesPerSec);
            currentAmps = table.getDoubleArray("MotorCurrentAmps", currentAmps);
            tempCelcius = table.getDoubleArray("MotorTempCelcius", tempCelcius);
        }
    }

}
