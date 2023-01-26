package frc.robot.maps.subsystems;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.math.controller.PIDController;

public class ArmMap {
    public ArmMap() {
    }

    private SmartMotorController extendMotor = new SmartMotorController();
    private final double SOFT_MAX_DISTANCE = 20;
    private final double SOFT_MIN_DISTANCE = 1;

    public ArmMap(SmartMotorController motor) {
        this.extendMotor = motor;
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

        @Override
        public void toLog(LogTable table) {
            // TODO Auto-generated method stub
            table.put("MotorSetpoint", setPoint);
            table.put("MotorDistanceInches", distanceInches);
            table.put("MotorVelocityInchesPerSec", velocityInchesPerSec);
            table.put("MotorCurrentAmps", currentAmps);
            table.put("MotorTempCelcius", tempCelcius);
        }

        @Override
        public void fromLog(LogTable table) {
            // TODO Auto-generated method stub
            setPoint = table.getDouble("MotorSetpoint", setPoint);
            distanceInches = table.getDouble("MotorDistanceInches", distanceInches);
            velocityInchesPerSec = table.getDouble("MotorVelocityInchesPerSec", velocityInchesPerSec);
            currentAmps = table.getDoubleArray("MotorCurrentAmps", currentAmps);
            tempCelcius = table.getDoubleArray("MotorTempCelcius", tempCelcius);
        }
    }

}
