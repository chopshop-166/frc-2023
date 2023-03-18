package frc.robot.maps.subsystems;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.chopshop166.chopshoplib.drive.MockSwerveModule;
import com.chopshop166.chopshoplib.drive.SwerveModule;
import com.chopshop166.chopshoplib.sensors.gyro.MockGyro;
import com.chopshop166.chopshoplib.sensors.gyro.PigeonGyro2;
import com.chopshop166.chopshoplib.sensors.gyro.SmartGyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.DrivePID;

/**
 * A hardware map suitable for a swerve drive.
 *
 * All Distances are in Meters
 */
public record SwerveDriveMap(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule rearLeft,
        SwerveModule rearRight, double maxDriveSpeedMetersPerSecond,
        double maxRotationRadianPerSecond, PigeonGyro2 gyro, DrivePID pid, Transform3d cameraPosition,
        String cameraName) {

    /** A distance to use for default values. */
    private static final double DEFAULT_DISTANCE_FROM_CENTER = 0.381;

    /** Construct a map that uses mocks for everything. */
    public SwerveDriveMap() {
        this(
                // Front Left
                new MockSwerveModule(new Translation2d(DEFAULT_DISTANCE_FROM_CENTER,
                        DEFAULT_DISTANCE_FROM_CENTER)),
                // Front Right
                new MockSwerveModule(new Translation2d(DEFAULT_DISTANCE_FROM_CENTER,
                        -DEFAULT_DISTANCE_FROM_CENTER)),
                // Rear Left
                new MockSwerveModule(new Translation2d(-DEFAULT_DISTANCE_FROM_CENTER,
                        DEFAULT_DISTANCE_FROM_CENTER)),
                // Rear Right
                new MockSwerveModule(new Translation2d(-DEFAULT_DISTANCE_FROM_CENTER,
                        -DEFAULT_DISTANCE_FROM_CENTER)),
                // Max speed (m/s)
                2.0,
                // Max rotation (rad/s)
                Math.PI,
                // Gyro
                new PigeonGyro2(1), new DrivePID(), new Transform3d(), "");
    }

    public void updateInputs(Data io) {
        io.frontLeft.update(this.frontLeft);
        io.frontRight.update(this.frontRight);
        io.rearLeft.update(this.rearLeft);
        io.rearRight.update(this.rearRight);

        io.gyroYawPositionDegrees = this.gyro.getAngle();
    }

    public static class Data implements LoggableInputs {
        public class SwerveModuleData {
            public String name;
            public double drivePositionMeters;
            public double velocityMetersPerSec;
            public double podAngle;
            public double[] driveCurrentAmps;
            public double[] driveTempC;
            public double[] steeringCurrentAmps;
            public double[] steeringTempC;
            public SwerveModuleState desiredState = new SwerveModuleState();
            public double steeringSetpoint;

            public SwerveModuleData(String name) {
                this.name = name;
            }

            public void toLog(LogTable table) {
                LogTable subTable = table.getSubtable(this.name);
                // Desired state (velocity/angle)
                subTable.put("DesiredVelocityMetresPerSec",
                        this.desiredState.speedMetersPerSecond);
                subTable.put("DesiredAngleDegrees", this.desiredState.angle.getDegrees());
                // actual state (Position/velocity/angle)
                subTable.put("DrivePositionMeters", this.drivePositionMeters);
                subTable.put("DriveVelocityMetersPerSec", this.velocityMetersPerSec);
                subTable.put("DriveAngleDegrees", this.podAngle);
                // Drive Motor params
                subTable.put("DriveCurrentAmps", this.driveCurrentAmps);
                subTable.put("DriveTempCelsius", this.driveTempC);
                // Steering Motor params
                subTable.put("SteeringCurrentAmps", this.steeringCurrentAmps);
                subTable.put("SteeringTempCelsius", this.steeringTempC);

                subTable.put("SteeringSetpoint", this.steeringSetpoint);
            }

            public void fromLog(LogTable table) {
                LogTable subTable = table.getSubtable(this.name);
                // Desired state (velocity/angle)
                this.desiredState.speedMetersPerSecond = subTable.getDouble("DesiredVelocityMetresPerSec",
                        this.desiredState.speedMetersPerSecond);
                this.desiredState.angle = Rotation2d.fromDegrees(subTable.getDouble("DesiredAngleDegrees",
                        this.desiredState.angle.getDegrees()));
                // actual state (Position/velocity/angle)
                this.drivePositionMeters = subTable.getDouble("DrivePositionMeters",
                        this.drivePositionMeters);

                this.velocityMetersPerSec = subTable.getDouble(
                        "DriveVelocityMetersPerSec", this.velocityMetersPerSec);
                this.podAngle = subTable.getDouble("DriveAngleDegrees", this.podAngle);
                this.driveCurrentAmps = subTable.getDoubleArray("DriveCurrentAmps", this.driveCurrentAmps);
                this.driveTempC = subTable.getDoubleArray("DriveTempCelsius", this.driveTempC);
                this.steeringCurrentAmps = subTable.getDoubleArray("SteeringCurrentAmps", this.steeringCurrentAmps);
                this.steeringTempC = subTable.getDoubleArray("SteeringTempCelsius", this.steeringTempC);
                this.steeringSetpoint = subTable.getDouble("SteeringSetpoint", this.steeringSetpoint);
            }

            public void update(SwerveModule module) {
                this.drivePositionMeters = module.getDistance();
                this.velocityMetersPerSec = module.getState().speedMetersPerSecond;
                this.podAngle = module.getAngle().getDegrees();
                this.driveCurrentAmps = module.getDriveMotor().getCurrentAmps();
                this.driveTempC = module.getDriveMotor().getTemperatureC();
                this.steeringCurrentAmps = module.getSteeringMotor().getCurrentAmps();
                this.steeringTempC = module.getSteeringMotor().getTemperatureC();
                module.setDesiredState(this.desiredState);
                this.steeringSetpoint = module.getSteeringMotor().get();
            }
        }

        public SwerveModuleData frontLeft = new SwerveModuleData("FrontLeft");
        public SwerveModuleData frontRight = new SwerveModuleData("FrontRight");
        public SwerveModuleData rearLeft = new SwerveModuleData("rearLeft");
        public SwerveModuleData rearRight = new SwerveModuleData("rearRight");

        public double gyroYawPositionDegrees = 0.0;

        @Override
        public void toLog(LogTable table) {
            frontLeft.toLog(table);
            frontRight.toLog(table);
            rearLeft.toLog(table);
            rearRight.toLog(table);

            table.put("GyroYawPositionDegrees", this.gyroYawPositionDegrees);
        }

        @Override
        public void fromLog(LogTable table) {
            frontLeft.fromLog(table);
            frontRight.fromLog(table);
            rearLeft.fromLog(table);
            rearRight.fromLog(table);

            this.gyroYawPositionDegrees = table.getDouble("GyroYawPositionDegrees", this.gyroYawPositionDegrees);
        }
    }
}
