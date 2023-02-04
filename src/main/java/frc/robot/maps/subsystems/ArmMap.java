package frc.robot.maps.subsystems;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.math.controller.PIDController;

public class ArmMap {
    public ArmMap() {
    }

    public PIDController pid = new PIDController(0, 0, 0);
    private SmartMotorController extendMotor = new SmartMotorController();
    public double softMaxDistance = 20;
    public double softMinDistance = 1;

    public ArmMap(SmartMotorController motor, double softMaxDistance, double softMinDistance, PIDController pid) {
        this.extendMotor = motor;
        this.softMaxDistance = softMaxDistance;
        this.softMinDistance = softMinDistance;
        this.pid = pid;
    }

    public void updateData(Data data) {
        extendMotor.set(data.setPoint);
        data.distanceInches = extendMotor.getEncoder().getDistance();
        data.velocityInchesPerSec = extendMotor.getEncoder().getRate();
        data.currentAmps = extendMotor.getCurrentAmps();
        data.tempCelcius = extendMotor.getTemperatureC();
    }

    public SmartMotorController getMotor() {
        return extendMotor;
    }

    public static class Data implements LoggableInputs {
        public double setPoint;
        public double distanceInches;
        public double velocityInchesPerSec;
        public double[] currentAmps;
        public double[] tempCelcius;

        @Override
        public void toLog(LogTable table) {
            table.put("MotorSetpoint", setPoint);
            table.put("MotorDistanceInches", distanceInches);
            table.put("MotorVelocityInchesPerSec", velocityInchesPerSec);
            table.put("MotorCurrentAmps", currentAmps);
            table.put("MotorTempCelcius", tempCelcius);
        }

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
