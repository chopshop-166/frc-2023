package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.race;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.RobotUtils;
import com.chopshop166.chopshoplib.commands.FunctionalWaitCommand;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.Field;
import frc.robot.Vision;
import frc.robot.maps.subsystems.SwerveDriveMap;
import frc.robot.maps.subsystems.SwerveDriveMap.Data;
import frc.robot.util.DrivePID;
import frc.robot.util.RotationPIDController;

public class Drive extends SmartSubsystemBase {

    SwerveDriveMap map;
    Data io;
    public final SwerveDriveKinematics kinematics;
    double maxDriveSpeedMetersPerSecond;
    double maxRotationRadiansPerSecond;
    double speedCoef = 1;
    double rotationCoef = 1;
    private final double TILT_VELOCITY_MAX = 4.0;
    private final double TILT_THRESHOLD = 4.0;
    private final double TILT_MAX_STOPPING = 1;

    private final double UNTIL_TIPPED_SPEED = 2;
    private final double UNTIL_NOT_TIPPED_SPEED = 0.5;
    private final double BALANCE_SPEED = 0.25;

    boolean isBlue = false;
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    BooleanPublisher autoBalanceState = inst.getBooleanTopic("Auto/Balance").publish();

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

    public Pose2d getPose() {
        return pose;
    }

    public void resetPose(Pose2d pose) {
        this.pose = pose;
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
        kinematics = new SwerveDriveKinematics(map.frontLeft().getLocation(), map.frontRight().getLocation(),
                map.rearLeft().getLocation(), map.rearRight().getLocation());
        maxDriveSpeedMetersPerSecond = map.maxDriveSpeedMetersPerSecond();
        maxRotationRadiansPerSecond = map.maxRotationRadianPerSecond();
        drivePID = map.pid();
        vision = new Vision(
                map.cameraName(), Field.getApriltagLayout(),
                map.cameraPosition(),
                this.map);
        correctionPID = new RotationPIDController(0.01, 0.00001, 0.0);
        rotationPID = drivePID.copyRotationPidController();
    }

    public Command rotateToAngle(Rotation2d angle, DoubleSupplier translateX, DoubleSupplier translateY) {
        return cmd().onExecute(() -> {
            double fb = rotationPID.calculate(pose.getRotation().getDegrees(), angle.getDegrees());
            Logger.getInstance().recordOutput("Rotaion PId", fb);
            move(translateX.getAsDouble(), translateY.getAsDouble(), fb);
        }).runsUntil(() -> Math.abs(pose.getRotation().getDegrees() - angle.getDegrees()) < 0.1)
                .onEnd(this::safeState);

    }

    public Command setSpeedCoef(double fac, double rotationfac) {
        return runOnce(() -> {
            speedCoef = fac;
            rotationCoef = rotationfac;
        });
    }

    Pose2d initialPose;

    public Command driveDistance(double distance, double speed, Rotation2d angle) {
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

    public Command driveAxis(double distance) {
        return cmd().onInitialize(() -> {
            startAxis = pose.getX();
        }).onExecute(() -> {
            Transform2d fb = drivePID.calculate(pose,
                    new Pose2d(startAxis + distance, 0, rotation0));
            move(fb.getX(), fb.getY(), 0 * fb.getRotation().getDegrees());
            Logger.recordOutput("Drive Ended", false);
        }).runsUntil(() -> Math.abs(pose.getX() - (startAxis + distance)) < 0.05).onEnd(() -> {
            move(0, 0, 0);
            System.out.println("Drive Command Stopped");
        });
    }

    private Pose2d relativeTarget = new Pose2d();

    public Command driveRelative(Translation2d translation, Rotation2d rotation2d, double timeoutSeconds) {
        return sequence(
                runOnce(() -> {
                    relativeTarget = new Pose2d(
                            pose.getX() + translation.getX(),
                            pose.getY() + translation.getY(),
                            rotation2d);
                    drivePID.reset(pose.getTranslation());
                }),
                race(new FunctionalWaitCommand(() -> timeoutSeconds),
                        driveTo(() -> relativeTarget, 0.01)));
    }

    public Command driveRelative(Translation2d translation, double angleDegrees, double timeoutSeconds) {
        return driveRelative(translation, Rotation2d.fromDegrees(angleDegrees), timeoutSeconds);
    }

    private void deadbandMove(final double xSpeed, final double ySpeed,
            final double rotation, boolean isRobotCentric) {

        var deadband = RobotUtils.scalingDeadband(
                (DriverStation.isFMSAttached()) ? 0.05 : 0.15);
        double rotationInput = deadband.applyAsDouble(rotation);
        double xInput = deadband.applyAsDouble(xSpeed);
        double yInput = deadband.applyAsDouble(ySpeed);

        SmartDashboard.putNumber("Rotation Correction Error", latestAngle - map.gyro().getAngle());

        if (Math.abs(rotationInput) < 0.1
                && !(Math.abs(xInput) < 0.1 && Math.abs(yInput) < 0.1)) {
            rotationInput = correctionPID.calculate(map.gyro().getAngle(), latestAngle);
            rotationInput = (Math.abs(rotationInput) > 0.02) ? rotationInput : 0;
            Logger.recordOutput("pidOutput", rotationInput);
            Logger.recordOutput("pidError", correctionPID.getError());
        } else {
            latestAngle = map.gyro().getAngle();
        }
        Logger.recordOutput("latestAngle", latestAngle);
        Logger.recordOutput("robotAngle", map.gyro().getAngle());

        final double translateXSpeed = xInput
                * maxDriveSpeedMetersPerSecond * speedCoef;
        final double translateYSpeed = yInput
                * maxDriveSpeedMetersPerSecond * speedCoef;
        final double rotationSpeed = rotationInput
                * maxRotationRadiansPerSecond * rotationCoef;
        _move(translateXSpeed, translateYSpeed, rotationSpeed, isRobotCentric);
    }

    public void move(final double xSpeed, final double ySpeed,
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

        // All the states

    }

    public void setModuleStates(SwerveModuleState[] moduleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxDriveSpeedMetersPerSecond);
        io.frontLeft.desiredState = moduleStates[0];
        io.frontRight.desiredState = moduleStates[1];
        io.rearLeft.desiredState = moduleStates[2];
        io.rearRight.desiredState = moduleStates[3];
    }

    public Command driveRaw(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotation) {
        return run(() -> move(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rotation.getAsDouble()));
    }

    public Command drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotation) {
        return run(() -> deadbandMove(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rotation.getAsDouble(), false));
    }

    public Command robotCentricDrive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotation) {
        return run(() -> deadbandMove(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rotation.getAsDouble(), true))
                .withName("Robot Centric Drive");
    }

    public Command driveTo(Supplier<Pose2d> targetPose, double tolerance) {
        return cmd().onExecute(() -> {
            Pose2d flipped = targetPose.get();
            Transform2d fb = drivePID.calculate(pose, flipped);
            Logger.recordOutput("DriveX", fb.getX());
            Logger.recordOutput("DriveY", fb.getY());
            move(fb.getX(), fb.getY(), -fb.getRotation().getDegrees());

            Logger.recordOutput("targetPose", flipped);
        }).runsUntil(() -> drivePID.isFinished(pose, targetPose.get(), tolerance)).onEnd(this::safeState);

    }

    public Command driveTo(Pose2d targetPose, double tolerance) {
        return driveTo(() -> targetPose, tolerance);
    }

    // Use DrivePID to drive to a target pose on the field
    public Command driveTo(Pose2d targetPose) {
        return driveTo(targetPose, 0.1);
    }

    public void resetTag() {
        vision.sawTag = false;
    }

    // Drive to a pre-determined grid position
    public Command driveTo(GridPosition gridPose) {
        return driveTo(gridPose.getPose(), 0.05);
    }

    // Reset odometry
    // public void resetOdometry(Pose2d pose) {
    // odemeter.reset
    // frontLeft.SwerveModulePosition(),
    // frontRight.SwerveModulePosition(),
    // rearLeft.SwerveModulePosition(),
    // rearRight.SwerveModulePosition()
    // , pose);
    //

    // Find the nearest grid position and line up with it
    public Command driveToNearest() {
        return new ProxyCommand(
                () -> {
                    Pose2d closestPose = GridPosition.values()[0].getPose();
                    for (GridPosition position : GridPosition.values()) {
                        // Only flip for the distance check, isn't needed for the actual driveTo since
                        // that also flips it
                        if (position.getPose().getTranslation()
                                .getDistance(pose.getTranslation()) < closestPose
                                        .getTranslation().getDistance(pose.getTranslation())) {
                            closestPose = position.getPose();
                        }

                    }

                    return driveTo(closestPose, 0.05);
                });

    }

    public Command driveUntilTipped(boolean forward) {
        return cmd().onExecute(() -> {
            if (forward) {
                Logger.recordOutput("AutoBalanceState", "tipping forward");
                move(0.0, -UNTIL_TIPPED_SPEED, 0.0);
            } else {
                Logger.recordOutput("AutoBalanceState", "tipping backward");
                move(0.0, UNTIL_TIPPED_SPEED, 0.0);
            }

        }).runsUntil(() -> Math.abs(this.getTilt()) > 4).onEnd((() -> {
            Logger.recordOutput("AutoBalanceState", "tipped");
        })).withTimeout(1.5);
    }

    public Command driveUntilNotTipped(boolean forward) {
        return cmd().onExecute(() -> {
            if (forward) {
                Logger.recordOutput("AutoBalanceState", "untipping forward");
                move(0.0, -UNTIL_NOT_TIPPED_SPEED, 0.0);
            } else {
                Logger.recordOutput("AutoBalanceState", "untipping backward");
                move(0.0, UNTIL_NOT_TIPPED_SPEED, 0.0);
            }

        }).runsUntil(() -> Math.abs(this.getTilt()) < 6).onEnd((() -> {
            Logger.recordOutput("AutoBalanceState", "not tipped");
        }));
    }

    public double getTilt() {
        return this.map.gyro().getRotation2d().getCos()
                * Units.radiansToDegrees(this.map.gyro().getRotation3d().getY())
                + this.map.gyro().getRotation2d().getSin()
                        * Units.radiansToDegrees(this.map.gyro().getRotation3d().getX());
    }

    public Command balance() {

        PersistenceCheck balancedCheck = new PersistenceCheck(50,
                () -> Math.abs(getTilt()) < 4);
        return cmd().onExecute(() -> {
            Rotation3d rotationVelocity = this.map.gyro().getRotationalVelocity();

            double angleVelocityDegreesPerSec = map.gyro().getRotation2d().getCos()
                    * Units.radiansToDegrees(rotationVelocity.getY())
                    + map.gyro().getRotation2d().getSin() * Units
                            .radiansToDegrees(rotationVelocity.getX());
            boolean shouldStop = (getTilt() < -TILT_MAX_STOPPING &&
                    angleVelocityDegreesPerSec > TILT_VELOCITY_MAX)
                    || (getTilt() > TILT_MAX_STOPPING && angleVelocityDegreesPerSec < -TILT_VELOCITY_MAX);

            Logger.recordOutput("Angle Velocity", angleVelocityDegreesPerSec);
            Logger.recordOutput("Pitch", getTilt());
            autoBalanceState.accept(shouldStop);

            if (shouldStop) {
                safeState();
                Logger.recordOutput("AutoBalanceState", "stopped");

            } else if (getTilt() > TILT_THRESHOLD) {
                move(0.0, BALANCE_SPEED, 0.0);

                Logger.recordOutput("AutoBalanceState", "backward");

            } else if (getTilt() < -TILT_THRESHOLD) {
                move(0.0, -BALANCE_SPEED, 0.0);

                Logger.recordOutput("AutoBalanceState", "forward");

            }

        }).runsUntil(balancedCheck).onEnd(() -> Logger.recordOutput("AutoBalanceState", "ended"));

    }

    public Command moveForDirectional(double xSpeed, double ySpeed, double seconds) {
        return race(
                driveRaw(() -> xSpeed, () -> ySpeed, () -> 0),
                new FunctionalWaitCommand(() -> seconds)).andThen(safeStateCmd());
    }

    public Command resetGyroCommand() {
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

    public Command setGyro180() {
        return runOnce(() -> {
            map.gyro().setAngle(180);
        });
    }

    public Command setPose(Pose2d pose) {
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
        isBlue = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;
        // This method will be called once per scheduler run
        // Use this for any background processing
        map.updateInputs(io);
        Logger.processInputs(getName(), io);
        pose = vision.update(isBlue);
        Logger.recordOutput("robotPose", pose);
        Logger.recordOutput("Pitch", getTilt());
    }

}