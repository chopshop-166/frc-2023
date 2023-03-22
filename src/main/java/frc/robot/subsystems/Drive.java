package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.Modifier;
import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.RobotUtils;
import com.chopshop166.chopshoplib.sensors.gyro.PigeonGyro2;
import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.sensors.gyro.PigeonGyro;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.Field;
import frc.robot.Vision;
import frc.robot.Robot;
import frc.robot.maps.subsystems.SwerveDriveMap;
import frc.robot.maps.subsystems.SwerveDriveMap.Data;
import frc.robot.util.DrivePID;
import frc.robot.util.RotationPIDController;

public class Drive extends SmartSubsystemBase {

    SwerveDriveMap map;
    Data io;
    Pigeon2 pigeonGyro;
    private final SwerveDriveKinematics kinematics;
    double maxDriveSpeedMetersPerSecond;
    double maxRotationRadiansPerSecond;
    double speedCoef = 1;
    double rotationCoef = 1;

    boolean isBlue = false;

    private Pose2d invertSide(Pose2d bluePose) {
        if (isBlue) {
            return bluePose;
        }
        return new Pose2d(
                Field.LENGTH - bluePose.getX(),
                bluePose.getY(),
                Rotation2d.fromDegrees(bluePose.getRotation().getDegrees() + 180));
    }

    private static final double blueX = 1.8;
    private static final Rotation2d rotation0 = Rotation2d.fromDegrees(0);
    private static final Rotation2d rotation180 = Rotation2d.fromDegrees(180);

    public enum GridPosition {

        CONE_5(
                new Pose2d(blueX, 5.42, rotation0)),

        CONE_4(
                new Pose2d(blueX, 2.95, rotation0)),

        CONE_3(
                new Pose2d(blueX, 3.56, rotation0)),

        CONE_2(
                new Pose2d(blueX, 2.36, rotation0)),

        CONE_1(
                new Pose2d(blueX, 1.82, rotation0)),

        CONE_0(
                new Pose2d(blueX, 0.6, rotation0));

        private Pose2d pose;

        private GridPosition(Pose2d pose) {
            this.pose = pose;
        }

        public Pose2d getPose() {
            return pose;
        }
    }

    private Vision vision;
    private Pose2d pose = new Pose2d();
    private final DrivePID drivePID;

    // Used for automatic alignment while driving
    private final RotationPIDController rotationPID;
    // Used for rotation correction while driving
    private final RotationPIDController correctionPID;

    private double latestAngle = 0;

    public Drive(SwerveDriveMap map) {
        this.map = map;
        this.map.gyro().reset();
        io = new Data();
        pigeonGyro = this.map.gyro().getRaw();
        kinematics = new SwerveDriveKinematics(map.frontLeft().getLocation(), map.frontRight().getLocation(),
                map.rearLeft().getLocation(), map.rearRight().getLocation());
        maxDriveSpeedMetersPerSecond = map.maxDriveSpeedMetersPerSecond();
        maxRotationRadiansPerSecond = map.maxRotationRadianPerSecond();
        drivePID = map.pid();
        vision = new Vision(
                map.cameraName(), Field.getApriltagLayout(),
                map.cameraPosition(),
                this.map);
        correctionPID = drivePID.copyRotationPidController();
        rotationPID = drivePID.copyRotationPidController();
    }

    public CommandBase rotateToAngle(Rotation2d angle, DoubleSupplier translateX, DoubleSupplier translateY) {
        return cmd().onExecute(() -> {
            double fb = rotationPID.calculate(pose.getRotation().getDegrees(), angle.getDegrees());
            move(translateX.getAsDouble(), translateY.getAsDouble(), fb);
        }).runsUntil(() -> Math.abs(pose.getRotation().getDegrees() - angle.getDegrees()) < 0.1)
                .onEnd(this::safeState);

    }

    public CommandBase setSpeedCoef(double fac, double rotationfac) {
        return runOnce(() -> {
            speedCoef = fac;
            rotationCoef = rotationfac;

        });
    }

    Pose2d initialPose;

    public CommandBase driveDistance(double distance, double speed, Rotation2d angle) {
        return cmd().onInitialize(() -> {
            initialPose = pose;
        }).onExecute(
                () -> {
                    move(angle.getSin() * speed, angle.getCos() * speed, 0);
                }).runsUntil(() -> pose.getTranslation().getDistance(initialPose.getTranslation()) > distance)
                .onEnd(() -> {
                    move(0, 0, 0);
                });

    }

    double startAxis = 0;

    public CommandBase driveAxis(double distance) {
        return cmd().onInitialize(() -> {
            startAxis = pose.getX();
        }).onExecute(() -> {
            Transform2d fb = drivePID.calculate(pose,
                    new Pose2d(startAxis + distance, 0, rotation0));
            move(fb.getX(), fb.getY(), 0 * fb.getRotation().getDegrees());
            Logger.getInstance().recordOutput("Drive Ended", false);
        }).runsUntil(() -> Math.abs(pose.getX() - (startAxis + distance)) < 0.05).onEnd(() -> {
            move(0, 0, 0);
            System.out.println("Drive Command Stopped");
        });
    }

    double targetComponent = 0;
    PIDController componentPID = new PIDController(0, 0, 0);

    public CommandBase driveX(double distanceMeters) {
        return cmd().onInitialize(() -> {
            componentPID = drivePID.copyTranslationPidController();
            componentPID.setTolerance(0.02);
            targetComponent = pose.getY() + distanceMeters;
            Logger.getInstance().recordOutput("targetX", targetComponent);
        }).onExecute(() -> {
            move(componentPID.calculate(pose.getY(), targetComponent), 0, 0);
            Logger.getInstance().recordOutput("poseX", pose.getY());
        }).runsUntil(componentPID::atSetpoint).onEnd(() -> move(0, 0, 0));
    }

    public CommandBase driveY(double distanceMeters) {
        return cmd().onInitialize(() -> {
            componentPID = drivePID.copyTranslationPidController();
            componentPID.setTolerance(0.02);
            targetComponent = pose.getX() + distanceMeters;
        }).onExecute(() -> {
            move(0, componentPID.calculate(pose.getX(), targetComponent), 0);
        }).runsUntil(componentPID::atSetpoint).onEnd(() -> move(0, 0, 0));
    }

    private void deadbandMove(final double xSpeed, final double ySpeed,
            final double rotation, boolean isRobotCentric) {

        var deadband = RobotUtils.scalingDeadband(0.15);
        double rotationInput = deadband.applyAsDouble(rotation);
        double xInput = deadband.applyAsDouble(xSpeed);
        double yInput = deadband.applyAsDouble(ySpeed);

        SmartDashboard.putNumber("Rotation Correction Error", latestAngle - map.gyro().getAngle());

        if (Math.abs(rotationInput) < 0.1
                && !(Math.abs(xInput) < 0.1 && Math.abs(yInput) < 0.1)) {
            rotationInput = correctionPID.calculate(latestAngle, map.gyro().getAngle());
            rotationInput = (Math.abs(rotationInput) > 0.02) ? rotationInput : 0;
            Logger.getInstance().recordOutput("pidOutput", rotationInput);
            Logger.getInstance().recordOutput("pidError", correctionPID.getError());
        } else {
            latestAngle = map.gyro().getAngle();
        }
        Logger.getInstance().recordOutput("latestAngle", latestAngle);
        Logger.getInstance().recordOutput("robotAngle", map.gyro().getAngle());

        final double translateXSpeed = xInput
                * maxDriveSpeedMetersPerSecond * speedCoef;
        final double translateYSpeed = yInput
                * maxDriveSpeedMetersPerSecond * speedCoef;
        final double rotationSpeed = rotationInput
                * maxRotationRadiansPerSecond * rotationCoef;
        _move(translateXSpeed, translateYSpeed, rotationSpeed, isRobotCentric);
    }

    private void move(final double xSpeed, final double ySpeed,
            final double rotation) {
        _move(xSpeed, ySpeed, rotation, false);
    }

    private void _move(final double xSpeed, final double ySpeed,
            final double rotation, boolean isRobotCentric) {

        // rotationOffset is temporary and startingRotation is set at the start
        ChassisSpeeds speeds;
        if (isRobotCentric) {
            speeds = new ChassisSpeeds(ySpeed, xSpeed, rotation);
        } else {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed,
                    rotation,
                    Rotation2d.fromDegrees(io.gyroYawPositionDegrees));
        }

        // Now use this in our kinematics
        final SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

        // Front left module state
        io.frontLeft.desiredState = moduleStates[0];

        // Front right module state
        io.frontRight.desiredState = moduleStates[1];

        // Back left module state
        io.rearLeft.desiredState = moduleStates[2];

        // Back right module state
        io.rearRight.desiredState = moduleStates[3];
    }

    public CommandBase driveRaw(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotation) {
        return run(() -> move(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rotation.getAsDouble()));
    }

    public CommandBase drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotation) {
        return run(() -> deadbandMove(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rotation.getAsDouble(), false));
    }

    public CommandBase robotCentricDrive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotation) {
        return run(() -> deadbandMove(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rotation.getAsDouble(), true))
                .withName("Robot Centric Drive");
    }

    public CommandBase driveTo(Pose2d targetPose, double tolerance) {
        return cmd().onExecute(() -> {
            Pose2d flipped = invertSide(targetPose);
            Transform2d fb = drivePID.calculate(pose, flipped);

            if (isBlue) {
                move(fb.getX(), fb.getY(), fb.getRotation().getDegrees());
            } else {

                move(-fb.getX(), -fb.getY(), -fb.getRotation().getDegrees());
            }

            Logger.getInstance().recordOutput("targetPose", flipped);
        }).runsUntil(() -> drivePID.isFinished(pose, invertSide(targetPose), tolerance)).onEnd(this::safeState);
    }

    // Use DrivePID to drive to a target pose on the field
    public CommandBase driveTo(Pose2d targetPose) {
        return driveTo(targetPose, 0.1);
    }

    public void resetTag() {
        vision.sawTag = false;
    }

    // Drive to a pre-determined grid position
    public CommandBase driveTo(GridPosition gridPose) {
        return driveTo(gridPose.getPose(), 0.05);
    }

    // Find the nearest grid position and line up with it
    public CommandBase driveToNearest() {
        return new ProxyCommand(
                () -> {
                    Pose2d closestPose = invertSide(GridPosition.values()[0].getPose());
                    for (GridPosition position : GridPosition.values()) {
                        // Only flip for the distance check, isn't needed for the actual driveTo since
                        // that also flips it
                        if (invertSide(position.getPose()).getTranslation()
                                .getDistance(pose.getTranslation()) < invertSide(closestPose)
                                        .getTranslation().getDistance(pose.getTranslation())) {
                            closestPose = position.getPose();
                        }
                    }

                    return driveTo(closestPose, 0.05);
                });

    }

    private enum robotDriveDirection {
        FORWARD,
        BACKWARD,
        STOPPED;
    }

    public CommandBase balance() {

        PersistenceCheck balancedCheck = new PersistenceCheck(15, () -> Math.abs(pigeonGyro.getPitch()) < 7);
        return cmd().onExecute(() -> {
            double[] xyz = new double[3];
            pigeonGyro.getRawGyro(xyz);
            double pitchVelocity = xyz[0];
            double rollVelocity = xyz[1];
            double yawVelocity = xyz[2];

            double velocityThresholdDegreesPerSec = 8.0;

            double angleVelocityDegreesPerSec = map.gyro().getRotation2d().getCos() * pitchVelocity
                    + map.gyro().getRotation2d().getSin() * rollVelocity;
            boolean shouldStop = (pigeonGyro.getPitch() < 0.0 &&
                    angleVelocityDegreesPerSec > velocityThresholdDegreesPerSec)
                    || (pigeonGyro.getPitch() > 0.0 && angleVelocityDegreesPerSec < -velocityThresholdDegreesPerSec);
            robotDriveDirection lastDroveDirection = robotDriveDirection.STOPPED;
            if (shouldStop) {
                safeState();
                Logger.getInstance().recordOutput("Driving:", "I'm stopped!");
            } else {
                if (Math.abs(angleVelocityDegreesPerSec) > 0.001) {
                    Logger.getInstance().recordOutput("Pitch Velocity I think happened", pitchVelocity);
                    if (lastDroveDirection == robotDriveDirection.FORWARD) {
                        move(0, -0.15, 0);
                        Logger.getInstance().recordOutput("Driving:", "backward-deriv");
                        Logger.getInstance().recordOutput("Balanced yet?", false);
                    } else if (lastDroveDirection == robotDriveDirection.BACKWARD) {
                        move(0, 0.15, 0);
                        Logger.getInstance().recordOutput("Driving:", "forward-deriv");
                        Logger.getInstance().recordOutput("Balanced yet?", false);
                    }
                } else if (pigeonGyro.getPitch() > 7) {
                    move(0.0, 0.15, 0.0);
                    lastDroveDirection = robotDriveDirection.FORWARD;
                    Logger.getInstance().recordOutput("Driving:", "forward");
                    Logger.getInstance().recordOutput("Balanced yet?", false);

                } else if (pigeonGyro.getPitch() < -7) {
                    move(0.0, -0.15, 0.0);
                    lastDroveDirection = robotDriveDirection.BACKWARD;
                    Logger.getInstance().recordOutput("Driving:", "backward");
                    Logger.getInstance().recordOutput("Balanced yet?", false);
                } else {
                    safeState();
                    lastDroveDirection = robotDriveDirection.STOPPED;
                    Logger.getInstance().recordOutput("Balanced yet?", true);

                }
            }

        }).runsUntil(balancedCheck).onEnd(() -> Logger.getInstance().recordOutput("Balanced yet?", true));
    }

    public CommandBase resetGyroCommand() {
        return cmd().onInitialize(() -> {
            resetGyro();
            resetTag();
        }).runsUntil(() -> {
            return true;
        }).runsWhenDisabled(true);
    }

    public void resetGyro() {
        map.gyro().reset();
        latestAngle = 0;
    }

    public CommandBase setGyro180() {
        return runOnce(() -> {
            map.gyro().setAngle(180);
        });
    }

    public CommandBase setPose(Pose2d pose) {
        return runOnce(() -> vision.setPose(pose));
    }

    @Override
    public void reset() {
        // Nothing to reset here
    }

    @Override
    public void safeState() {
        move(0, 0, 0);
    }

    @Override
    public void periodic() {
        isBlue = DriverStation.getAlliance() == Alliance.Blue;
        // This method will be called once per scheduler run
        // Use this for any background processing
        map.updateInputs(io);
        Logger.getInstance().processInputs(getName(), io);
        pose = vision.update(isBlue);
        Logger.getInstance().recordOutput("robotPose", pose);
    }
}