package frc.robot.maps;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.chopshop166.chopshoplib.drive.SDSSwerveModule;
import com.chopshop166.chopshoplib.drive.SDSSwerveModule.Configuration;
import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.motors.CSSparkMax;
import com.chopshop166.chopshoplib.sensors.MockEncoder;
import com.chopshop166.chopshoplib.sensors.gyro.PigeonGyro;
import com.chopshop166.chopshoplib.states.PIDValues;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.maps.subsystems.ArmExtendMap;
import frc.robot.maps.subsystems.ArmRotateMap;
import frc.robot.maps.subsystems.LedMap;
import frc.robot.maps.subsystems.SwerveDriveMap;
import frc.robot.util.DrivePID;

@RobotMapFor("Valkyrie")
public class Valkyrie extends RobotMap {

    @Override
    public LedMap getLedMap() {
        return new LedMap(0, 30);
    }

    @Override
    public ArmRotateMap getArmRotateMap() {
        CSSparkMax motor = new CSSparkMax(18, MotorType.kBrushless);
        ProfiledPIDController pid = new ProfiledPIDController(0.01, 0, 0, new Constraints(60, 10));
        DutyCycleEncoder mockArmEncoder = new DutyCycleEncoder(18);
        pid.setTolerance(1);
        return new ArmRotateMap(motor, 1, 10, 1, 10, 0, pid, new MockEncoder(), 0, 0);
    }

    @Override
    public ArmExtendMap getArmMap() {
        CSSparkMax motor = new CSSparkMax(9, MotorType.kBrushless);
        motor.getMotorController().setIdleMode(IdleMode.kBrake);
        ProfiledPIDController pidController = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));
        pidController.setTolerance(4);
        return new ArmExtendMap(motor, 400, 20, 400, 20, pidController, 46.654, 42.3);
    }

    @Override
    public void setupLogging() {
        Logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        Logger.recordMetadata("RobotMap", this.getClass().getSimpleName());
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    }

    @Override
    public SwerveDriveMap getDriveMap() {

        final double MODULE_OFFSET_XY = Units.inchesToMeters(12.375);
        final PigeonGyro pigeonGyro = new PigeonGyro(new PigeonIMU(0));

        final CSSparkMax frontLeftSteer = new CSSparkMax(4, MotorType.kBrushless);
        final CSSparkMax frontRightSteer = new CSSparkMax(8, MotorType.kBrushless);
        final CSSparkMax rearLeftSteer = new CSSparkMax(2, MotorType.kBrushless);
        final CSSparkMax rearRightSteer = new CSSparkMax(6, MotorType.kBrushless);

        frontLeftSteer.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        frontRightSteer.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        rearLeftSteer.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        rearRightSteer.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);

        // Configuration for MK4 with L2 speeds
        Configuration MK4_V2 = new Configuration(SDSSwerveModule.MK4_V2.gearRatio,
                SDSSwerveModule.MK4_V2.wheelDiameter, new PIDValues(0.011, 0.00, 0.0002));

        // All Distances are in Meters
        // Front Left Module
        final CANCoder encoderFL = new CANCoder(2);
        encoderFL.configMagnetOffset(-195.381);
        encoderFL.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule frontLeft = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                encoderFL, frontLeftSteer, new CSSparkMax(3, MotorType.kBrushless),
                MK4_V2);

        // Front Right Module
        final CANCoder encoderFR = new CANCoder(4);
        encoderFR.configMagnetOffset(-304.189 + 180);
        encoderFR.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule frontRight = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                encoderFR, frontRightSteer, new CSSparkMax(7,
                        MotorType.kBrushless),
                MK4_V2);

        // Rear Left Module
        final CANCoder encoderRL = new CANCoder(1);
        encoderRL.configMagnetOffset(-298.213);
        encoderRL.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule rearLeft = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                encoderRL, rearLeftSteer, new CSSparkMax(1,
                        MotorType.kBrushless),
                MK4_V2);

        // Rear Right Module
        final CANCoder encoderRR = new CANCoder(3);
        encoderRR.configMagnetOffset(-168.223 + 180);
        encoderRR.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule rearRight = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                encoderRR, rearRightSteer, new CSSparkMax(5,
                        MotorType.kBrushless),
                MK4_V2);

        final double maxDriveSpeedMetersPerSecond = Units.feetToMeters(12);

        final double maxRotationRadianPerSecond = Math.PI;

        final DrivePID pid = new DrivePID(
                1.2, 0.002, 0.0,
                0.02, 0.00001, 0,
                new Constraints(1.5, 2.5));

        final Transform3d cameraPosition = new Transform3d(
                // These probably need to be refined
                new Translation3d(
                        Units.inchesToMeters(
                                2.44),
                        Units.inchesToMeters(
                                -7.25),
                        Units.inchesToMeters(
                                25)),
                new Rotation3d());

        return new SwerveDriveMap(frontLeft, frontRight, rearLeft, rearRight,
                maxDriveSpeedMetersPerSecond,
                maxRotationRadianPerSecond, pigeonGyro, pid, cameraPosition, "eyes");

    }
}
