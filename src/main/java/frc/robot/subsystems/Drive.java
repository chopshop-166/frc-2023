package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.Modifier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

    public enum GridPosition {

        BLUE_CONE_4(new Pose2d(1.536, 1.807, Rotation2d.fromDegrees(162.722)));
        // BLUE_CONE_3(new Pose2d(1.708, 3.369, Rotation2d.fromDegrees(188.314))),
        // BLUE_CONE_2(new Pose2d(1.605, 3.905, Rotation2d.fromDegrees(166.823)));

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
    private Field2d field = new Field2d();

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

    private void deadbandMove(final double xSpeed, final double ySpeed,
            final double rotation) {

        final Modifier deadband = Modifier.deadband(0.15);

        double rotationInput = deadband.applyAsDouble(rotation);

        SmartDashboard.putNumber("Rotation Correction Error", latestAngle - map.gyro().getAngle());

        if (Math.abs(rotationInput) < 0.1
                && !(deadband.applyAsDouble(xSpeed) < 0.1 && deadband.applyAsDouble(ySpeed) < 0.1)) {
            rotationInput = correctionPID.calculate(latestAngle, map.gyro().getAngle());
            rotationInput = (Math.abs(rotationInput) > 0.02) ? rotationInput : 0;
            Logger.getInstance().recordOutput("pidOutput", rotationInput);
            Logger.getInstance().recordOutput("pidError", correctionPID.getError());
        } else {
            latestAngle = map.gyro().getAngle();
        }
        Logger.getInstance().recordOutput("latestAngle", latestAngle);
        Logger.getInstance().recordOutput("robotAngle", map.gyro().getAngle());

        final double translateXSpeed = deadband.applyAsDouble(xSpeed)
                * maxDriveSpeedMetersPerSecond * speedCoef;
        final double translateYSpeed = deadband.applyAsDouble(ySpeed)
                * maxDriveSpeedMetersPerSecond * speedCoef;
        final double rotationSpeed = rotationInput
                * maxRotationRadiansPerSecond * rotationCoef;
        move(translateXSpeed, translateYSpeed, rotationSpeed);
    }

    private void move(final double xSpeed, final double ySpeed,
            final double rotation) {

        // rotationOffset is temporary and startingRotation is set at the start
        final ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed,
                rotation,
                Rotation2d.fromDegrees(io.gyroYawPositionDegrees));

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
        return run(() -> deadbandMove(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rotation.getAsDouble()));
    }

    public CommandBase driveTo(Pose2d targetPose, double tolerance) {
        return cmd().onExecute(() -> {
            Pose2d flipped = invertSide(targetPose);
            Transform2d fb = drivePID.calculate(pose, flipped);
            move(fb.getX(), fb.getY(), fb.getRotation().getDegrees());

            double debugPose[] = new double[] { flipped.getX(), flipped.getY(),
                    flipped.getRotation().getDegrees() };
            SmartDashboard.putNumberArray("Target Pose", debugPose);
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

                    return driveTo(closestPose);
                });

    }

    public void resetGyro() {
        map.gyro().reset();
        latestAngle = 0;
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

        pose = vision.update();
        field.setRobotPose(pose);
        SmartDashboard.putData(field);
    }
}