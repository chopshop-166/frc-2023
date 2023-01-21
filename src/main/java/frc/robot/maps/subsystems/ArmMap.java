package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.math.controller.PIDController;

public class ArmMap {
    public ArmMap() {
    }

    private SmartMotorController motor = new SmartMotorController();

    public ArmMap(SmartMotorController motor) {
        this.motor = motor;
    }

    public class Data implements LoggableInputs {
        double setPoint;
        double distanceInches;
        double velocityInchesPerSec;
        double[] currentAmps;
        double[] tempCelcius;

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
            currentAmps = table.getDouble("MotorCurrentAmps", currentAmps);
            tempCelcius = table.getDouble("MotorTempCelcius", tempCelcius);
        }
    }

}
