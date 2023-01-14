package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class DrivePID {
    private PIDController xPid;
    private PIDController yPid;
    private PIDController anglePid;

    public DrivePID(double positionP, double positionI, double positionD, double angleP, double angleI, double angleD) {
        xPid = new PIDController(positionP, positionI, positionD);
        yPid = new PIDController(positionP, positionI, positionD);
        anglePid = new PIDController(angleP, angleI, angleD);
    }

    public Transform2d calculate(Pose2d currentPose, Pose2d targetPose) {
        Transform2d error = targetPose.minus(currentPose);
        return new Transform2d(
                new Translation2d(
                        xPid.calculate(error.getX()),
                        yPid.calculate(error.getY())),
                Rotation2d.fromDegrees(anglePid.calculate(error.getRotation().getDegrees())));
    }

    public boolean isFinished(Pose2d currentPose, Pose2d targetPose, double delta) {
        Transform2d error = targetPose.minus(currentPose);
        return error.getX() < delta && error.getY() < delta && error.getRotation().getDegrees() < delta;
    }

}
