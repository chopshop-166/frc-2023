package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.drive.SwerveDriveMap;
import com.chopshop166.chopshoplib.drive.SwerveModule;
import com.chopshop166.chopshoplib.motors.Modifier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Field;
import frc.robot.Vision;
import frc.robot.util.DrivePID;

public class Drive extends SmartSubsystemBase {

    private SwerveDriveMap map;

    private Vision vision;
    private Pose2d pose = new Pose2d();

    private final double maxDriveSpeedMetersPerSecond;
    private final double maxRotationRadiansPerSecond;

    private final SwerveDriveKinematics kinematics;
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule rearLeft;
    private final SwerveModule rearRight;

    private final DrivePID drivePID = new DrivePID(0, 0, 0, 0, 0, 0);

    public Drive(
            SwerveDriveMap map) {
        this.map = map;
        vision = new Vision(
                "photonvision", Field.getApriltagLayout(),
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(0),
                                Units.inchesToMeters(0),
                                Units.inchesToMeters(0)),
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0))),
                this.map);
        maxDriveSpeedMetersPerSecond = map.maxDriveSpeedMetersPerSecond();
        maxRotationRadiansPerSecond = map.maxRotationRadianPerSecond();
        frontLeft = map.frontLeft();
        frontRight = map.frontRight();
        rearLeft = map.rearLeft();
        rearRight = map.rearRight();
        kinematics = new SwerveDriveKinematics(frontLeft.getLocation(), frontRight.getLocation(),
                rearLeft.getLocation(), rearRight.getLocation());
    }

    public CommandBase driveTo(Pose2d targetPose) {
        return cmd().onExecute(() -> {
            Transform2d fb = drivePID.calculate(pose, targetPose);
            updateSwerveSpeedAngle(fb::getX, fb::getY, fb.getRotation()::getDegrees);
        }).runsUntil(() -> drivePID.isFinished(pose, targetPose, 0.01));
    }

    public CommandBase fieldCentricDrive(final DoubleSupplier translateX, final DoubleSupplier translateY,
            final DoubleSupplier rotation) {
        return run(() -> updateSwerveSpeedAngle(translateX, translateY, rotation));

    }

    private void updateSwerveSpeedAngle(final DoubleSupplier translateX, final DoubleSupplier translateY,
            final DoubleSupplier rotation) {
        final Modifier deadband = Modifier.deadband(0.15);
        final double translateXSpeed = deadband.applyAsDouble(translateX.getAsDouble()) * maxDriveSpeedMetersPerSecond;
        final double translateYSpeed = deadband.applyAsDouble(translateY.getAsDouble()) * maxDriveSpeedMetersPerSecond;
        final double rotationSpeed = deadband.applyAsDouble(rotation.getAsDouble()) * maxRotationRadiansPerSecond;

        // rotationOffset is temporary and startingRotation is set at the start
        final ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translateYSpeed, translateXSpeed,
                rotationSpeed, Rotation2d.fromDegrees(map.gyro().getAngle()));

        // Now use this in our kinematics
        final SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

        // Front left module state
        frontLeft.setDesiredState(moduleStates[0]);

        // Front right module state
        frontRight.setDesiredState(moduleStates[1]);

        // Back left module state
        rearLeft.setDesiredState(moduleStates[2]);

        // Back right module state
        rearRight.setDesiredState(moduleStates[3]);
    }

    @Override
    public void reset() {
        // Nothing to reset here
    }

    @Override
    public void safeState() {

    }

    @Override
    public void periodic() {
        pose = vision.update();
    }
}