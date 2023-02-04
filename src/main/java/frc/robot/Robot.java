package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.Autonomous;
import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap;
// $Imports$
import frc.robot.subsystems.Intake;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Led;

public class Robot extends CommandRobot {

    private RobotMap map = getMapForName("OffAxis", RobotMap.class, "frc.robot.maps");
    private ButtonXboxController driveController = new ButtonXboxController(0);
    // $Subsystems$
    Intake intake = new Intake(map.getIntakeMap());

    private Drive drive = new Drive(map.getDriveMap());
    private Led led = new Led(map.getLedMap());

    @Autonomous(defaultAuto = true)
    public CommandBase exampleAuto = auto.exampleAuto();

    // @Autonomous(defaultAuto = true)
    // public CommandBase exampleAuto = drive.driveTo(new Pose2d(1.29805263547889,
    // 2.709488526850264, new Rotation2d()))
    // .withName("Test");

    @Override
    public void teleopInit() {

    }

    @Override
    public void robotInit() {
        super.robotInit();

        Logger.getInstance().recordMetadata("ProjectName", "FRC-2023"); // Set a metadata value
        map.SetupLogging();
        if (!isReal()) {
            setUseTiming(false); // Run as fast as possible
        }
        // Start logging! No more data receivers, replay sources, or metadata values may
        // be added.
        Logger.getInstance().start();
    }

    @Override
    public void configureButtonBindings() {
        driveController.rightBumper().onTrue(drive.setSpeedCoef(0.5)).onFalse(drive.setSpeedCoef(1));
    }

    @Override
    public void populateDashboard() {

    }

    @Override
    public void setDefaultCommands() {
        drive.setDefaultCommand(
                drive.drive(driveController::getLeftX, driveController::getLeftY, driveController::getRightX));

        led.setDefaultCommand(led.colorAlliance());
    }
}