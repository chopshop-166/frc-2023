package frc.robot.maps;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.chopshop166.chopshoplib.drive.SDSSwerveModule;
import com.chopshop166.chopshoplib.drive.SDSSwerveModule.Configuration;
import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.motors.CSSparkMax;
import com.chopshop166.chopshoplib.motors.CSTalonSRX;
import com.chopshop166.chopshoplib.pneumatics.RevDSolenoid;
import com.chopshop166.chopshoplib.sensors.MockColorSensor;
import com.chopshop166.chopshoplib.sensors.gyro.PigeonGyro;
import com.chopshop166.chopshoplib.states.PIDValues;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.maps.subsystems.ArmMap;
import frc.robot.maps.subsystems.ArmRotateMap;
import frc.robot.maps.subsystems.IntakeData;
import frc.robot.maps.subsystems.SwerveDriveMap;
import frc.robot.util.DrivePID;

@RobotMapFor("Honre")
public class HonreMap extends RobotMap {

    @Override
    public SwerveDriveMap getDriveMap() {
        // Value taken from CAD as offset from center of module base pulley to center
        // of the robot

        final double MODULE_OFFSET_XY = Units.inchesToMeters(8.89);
        final PigeonGyro pigeonGyro = new PigeonGyro(new PigeonIMU(1));

        final CSSparkMax frontLeftSteer = new CSSparkMax(8, MotorType.kBrushless);
        final CSSparkMax frontRightSteer = new CSSparkMax(6, MotorType.kBrushless);
        final CSSparkMax rearLeftSteer = new CSSparkMax(4, MotorType.kBrushless);
        final CSSparkMax rearRightSteer = new CSSparkMax(2, MotorType.kBrushless);

        frontLeftSteer.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        frontRightSteer.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        rearLeftSteer.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        frontRightSteer.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);

        // Configuration for MK4i with L2 speeds
        Configuration MK4i_L2 = new Configuration(SDSSwerveModule.MK4_V2.gearRatio,
                SDSSwerveModule.MK4_V2.wheelDiameter, new PIDValues(0.002568, 0.00, 0.0001));

        // All Distances are in Meters
        // Front Left Module
        final CANCoder encoderFL = new CANCoder(4);
        encoderFL.configMagnetOffset(34.365);
        encoderFL.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule frontLeft = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                encoderFL, frontLeftSteer, new CSSparkMax(7, MotorType.kBrushless),
                MK4i_L2);

        // Front Right Module
        final CANCoder encoderFR = new CANCoder(3);
        encoderFR.configMagnetOffset(52.910);
        encoderFR.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule frontRight = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                encoderFR, frontRightSteer, new CSSparkMax(5,
                        MotorType.kBrushless),
                MK4i_L2);

        // Rear Left Module
        final CANCoder encoderRL = new CANCoder(2);
        encoderRL.configMagnetOffset(-92.285);
        encoderRL.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule rearLeft = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                encoderRL, rearLeftSteer, new CSSparkMax(3,
                        MotorType.kBrushless),
                MK4i_L2);

        // Rear Right Module
        final CANCoder encoderRR = new CANCoder(1);
        encoderRR.configMagnetOffset(25.049);
        encoderRR.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule rearRight = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                encoderRR, rearRightSteer, new CSSparkMax(1,
                        MotorType.kBrushless),
                MK4i_L2);

        final double maxDriveSpeedMetersPerSecond = Units.feetToMeters(10);

        final double maxRotationRadianPerSecond = Math.PI;

        final DrivePID pid = new DrivePID(0.2, 0, 0.05, 0.001, 0, 0);

        final Transform3d cameraPosition = new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(
                                2.44),
                        Units.inchesToMeters(
                                7.25),
                        Units.inchesToMeters(
                                25)),
                new Rotation3d());

        return new SwerveDriveMap(frontLeft, frontRight, rearLeft, rearRight,
                maxDriveSpeedMetersPerSecond,
                maxRotationRadianPerSecond, pigeonGyro, pid, cameraPosition, "eyes");
    }

    @Override
    public IntakeData.Map getIntakeMap() {
        CSTalonSRX intakeMotor = new CSTalonSRX(9);
        intakeMotor.setInverted(true);
        RevDSolenoid intakeSolenoid = new RevDSolenoid(8, 9);

        return new IntakeData.Map(intakeMotor, intakeSolenoid, new MockColorSensor());

    }

    @Override
    public ArmRotateMap getArmRotateMap() {
        CSSparkMax motor = new CSSparkMax(10, MotorType.kBrushless);
        motor.getMotorController().setInverted(false);
        motor.getEncoder().setPositionScaleFactor(1.125);
        motor.getEncoder().setVelocityScaleFactor(1.125);
        return new ArmRotateMap(motor, 85, 10, 115, 0, new PIDController(0, 0, 0));

    }

    @Override
    public ArmMap getArmMap() {
        CSSparkMax motor = new CSSparkMax(11, MotorType.kBrushless);
        motor.getMotorController().setInverted(true);
        motor.getEncoder().setPositionScaleFactor((1.273 * Math.PI) / 10);
        motor.getEncoder().setVelocityScaleFactor((1.273 * Math.PI) / 10);
        return new ArmMap(motor, 19, 1, 19.9, 0, new PIDController(0, 0, 0));
    }

    @Override
    public void setupLogging() {
        Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
        Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        Logger.getInstance().recordMetadata("RobotMap", this.getClass().getSimpleName());
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    }
}
