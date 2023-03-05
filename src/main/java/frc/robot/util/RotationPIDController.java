package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;

public class RotationPIDController {
    private PIDController pid;

    public RotationPIDController(double kP, double kI, double kD) {
        pid = new PIDController(kP, kI, kD);
    }

    public double calculate(double measurementDegrees, double setpointDegrees) {
        double angleError = -(setpointDegrees - measurementDegrees);

        if (Math.abs(angleError) > 180.0) {
            angleError = Math.signum(-angleError) * (360.0 - Math.abs(angleError));
        }
        return pid.calculate(angleError);
    }

    public double getError() {
        return pid.getPositionError();
    }

    public double getP() {
        return pid.getP();
    }

    public double getI() {
        return pid.getI();
    }

    public double getD() {
        return pid.getD();
    }
}
