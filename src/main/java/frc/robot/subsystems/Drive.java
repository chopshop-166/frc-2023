package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.Modifier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.Field;
import frc.robot.Vision;
import frc.robot.maps.subsystems.SwerveDriveMap;
import frc.robot.maps.subsystems.SwerveDriveMap.Data;
import frc.robot.util.DrivePID;

public class Drive extends SmartSubsystemBase {

    SwerveDriveMap map;
    Data io;
    private final SwerveDriveKinematics kinematics;
    double maxDriveSpeedMetersPerSecond;
    double maxRotationRadiansPerSecond;
    double speedCoef = 1;

    public enum GridPosition {

        TEST(new Pose2d());

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

    private final PIDController rotationPID;

    public Drive(SwerveDriveMap map) {
        this.map = map;
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

        rotationPID = drivePID.copyRotationPidController();
    }

    public CommandBase rotateToAngle(Rotation2d angle, DoubleSupplier translateX, DoubleSupplier translateY) {
        return cmd().onExecute(() -> {
            double fb = rotationPID.calculate(pose.getRotation().getDegrees(), angle.getDegrees());
            move(translateX.getAsDouble(), translateY.getAsDouble(), fb);
        }).runsUntil(() -> Math.abs(pose.getRotation().getDegrees() - angle.getDegrees()) < 0.1)
                .onEnd(this::safeState);

    }

    public CommandBase setSpeedCoef(double fac) {
        return runOnce(() -> {
            speedCoef = fac;
        });
    }

    private void move(final double xSpeed, final double ySpeed,
            final double rotation) {

        final Modifier deadband = Modifier.deadband(0.15);
        final double translateXSpeed = deadband.applyAsDouble(xSpeed)
                * maxDriveSpeedMetersPerSecond * speedCoef;
        final double translateYSpeed = deadband.applyAsDouble(ySpeed)
                * maxDriveSpeedMetersPerSecond * speedCoef;
        final double rotationSpeed = deadband.applyAsDouble(rotation)
                * maxRotationRadiansPerSecond * speedCoef;

        // rotationOffset is temporary and startingRotation is set at the start
        final ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translateYSpeed, translateXSpeed,
                rotationSpeed,
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

    public CommandBase drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotation) {
        return run(() -> move(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rotation.getAsDouble()));
    }

    // Use DrivePID to drive to a target pose on the field
    public CommandBase driveTo(Pose2d targetPose) {
        return cmd().onExecute(() -> {
            Transform2d fb = drivePID.calculate(pose, targetPose);
            move(fb.getX(), fb.getY(), fb.getRotation().getDegrees());
        }).runsUntil(() -> drivePID.isFinished(pose, targetPose, 0.005)).onEnd(this::safeState);
    }

    // Drive to a pre-determined grid position
    public CommandBase driveTo(GridPosition gridPose) {
        return driveTo(gridPose.getPose());
    }

    // Find the nearest grid position and line up with it
    public CommandBase driveToNearest() {
        return new ProxyCommand(
                () -> {
                    Pose2d closestPose = GridPosition.values()[0].getPose();
                    for (GridPosition position : GridPosition.values()) {
                        if (position.getPose().getTranslation().getDistance(pose.getTranslation()) < closestPose
                                .getTranslation().getDistance(pose.getTranslation())) {
                            closestPose = position.getPose();
                        }
                    }

                    return driveTo(closestPose);
                });

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
        // This method will be called once per scheduler run
        // Use this for any background processing
        map.updateInputs(io);
        Logger.getInstance().processInputs(getName(), io);

        pose = vision.update();
        field.setRobotPose(pose);
        SmartDashboard.putData(field);
    }
}