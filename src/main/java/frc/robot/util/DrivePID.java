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

    public DrivePID() {
        this(0, 0, 0, 0, 0, 0);
    }

    /**
     * Wrapper for three PIDController objects, for each cardinal direction and
     * rotation.
     * 
     * @param positionP kP for x and y positions
     * @param positionI kI for x and y positions
     * @param positionD kD for x and y positions
     * @param angleP    kP for angle
     * @param angleI    kI for angle
     * @param angleD    kD for angle
     */
    public DrivePID(double positionP, double positionI, double positionD, double angleP, double angleI, double angleD) {
        xPid = new PIDController(positionP, positionI, positionD);
        yPid = new PIDController(positionP, positionI, positionD);
        anglePid = new PIDController(angleP, angleI, angleD);
    }

    /**
     * Get the direction to move/rotate in based on a current pose and target pose
     * 
     * @param currentPose The current pose
     * @param targetPose  The target pose
     * @return The transformation needed to move in the direction of the target pose
     */
    public Transform2d calculate(Pose2d currentPose, Pose2d targetPose) {
        return new Transform2d(
                new Translation2d(
                        xPid.calculate(currentPose.getX(), targetPose.getX()),
                        yPid.calculate(currentPose.getY(), targetPose.getY())),
                Rotation2d.fromDegrees(anglePid.calculate(currentPose.getRotation().getDegrees(),
                        targetPose.getRotation().getDegrees())));
    }

    /**
     * Checks if the current pose matches the target pose
     * 
     * @param currentPose The current pose
     * @param targetPose  The target pose
     * @param delta       The acceptable difference between the current and target
     *                    poses
     * @return if current pose and target pose match
     */
    public boolean isFinished(Pose2d currentPose, Pose2d targetPose, double delta) {
        Transform2d error = targetPose.minus(currentPose);
        return error.getX() < delta && error.getY() < delta && error.getRotation().getDegrees() < delta;
    }

}
