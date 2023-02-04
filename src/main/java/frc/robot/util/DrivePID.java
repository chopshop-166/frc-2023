package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
                        yPid.calculate(targetPose.getY(), currentPose.getY()),
                        xPid.calculate(targetPose.getX(), currentPose.getX())),
                Rotation2d.fromDegrees(anglePid.calculate(currentPose.getRotation().getDegrees(),
                        targetPose.getRotation().getDegrees())));
    }

    /**
     * Checks if the current pose matches the target pose
     * 
     * @param currentPose The current pose
     * @param targetPose  The target pose
     * @param deadband    The acceptable difference between the current and target
     *                    poses
     * @return if current pose and target pose match
     */
    public boolean isFinished(Pose2d currentPose, Pose2d targetPose, double deadband) {
        Transform2d error = targetPose.minus(currentPose);

        SmartDashboard.putNumberArray("PID Error",
                new double[] { error.getX(), error.getY(), error.getRotation().getRadians() });

        return (error.getX() < deadband) && (error.getY() < deadband) && error.getRotation().getRadians() < deadband;
    }

    /**
     * Get a PID Controller using the values from the translation PID Controller
     * 
     * @return the translation PID Controller
     */
    public PIDController getTranslationPidController() {
        return new PIDController(xPid.getP(), xPid.getI(), xPid.getD());
    }

    /**
     * Get a PID Controller using the values from the rotation PID Controller
     * 
     * @return the rotation PID Controller
     */
    public PIDController getRotationPidController() {
        return new PIDController(anglePid.getP(), anglePid.getI(), anglePid.getD());
    }

}
