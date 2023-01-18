package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.Modifier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.subsystems.SwerveDriveMap;
import frc.robot.maps.subsystems.SwerveDriveMap.Data;

public class Drive extends SmartSubsystemBase {

    SwerveDriveMap map;
    Data io;
    private final SwerveDriveKinematics kinematics;
    double maxDriveSpeedMetersPerSecond;
    double maxRotationRadiansPerSecond;

    public Drive(SwerveDriveMap map) {
        this.map = map;
        io = new Data();
        kinematics = new SwerveDriveKinematics(map.frontLeft().getLocation(), map.frontRight().getLocation(),
                map.rearLeft().getLocation(), map.rearRight().getLocation());
        maxDriveSpeedMetersPerSecond = map.maxDriveSpeedMetersPerSecond();
        maxRotationRadiansPerSecond = map.maxRotationRadianPerSecond();
    }

    public CommandBase drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotation) {
        return run(() -> {
            final Modifier deadband = Modifier.deadband(0.15);
            final double translateXSpeed = deadband.applyAsDouble(xSpeed.getAsDouble())
                    * maxDriveSpeedMetersPerSecond;
            final double translateYSpeed = deadband.applyAsDouble(ySpeed.getAsDouble())
                    * maxDriveSpeedMetersPerSecond;
            final double rotationSpeed = deadband.applyAsDouble(rotation.getAsDouble()) * maxRotationRadiansPerSecond;

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
        });
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
        // This method will be called once per scheduler run
        // Use this for any background processing
        map.updateInputs(io);
        Logger.getInstance().processInputs(getName(), io);
    }
}