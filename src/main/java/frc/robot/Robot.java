package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.Autonomous;
import com.chopshop166.chopshoplib.RobotUtils;
import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap;
import frc.robot.subsystems.Drive;
// $Imports$
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmRotate;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Led;

public class Robot extends CommandRobot {

    private RobotMap map = getMapForName("FrostBite", RobotMap.class, "frc.robot.maps");
    private ButtonXboxController driveController = new ButtonXboxController(0);
    private ButtonXboxController copilotController = new ButtonXboxController(1);

    // $Subsystems$
    private Arm arm = new Arm(map.getArmMap());

    Intake intake = new Intake(map.getIntakeMap());

    private Drive drive = new Drive(map.getDriveMap());
    private Led led = new Led(map.getLedMap());
    private ArmRotate armRotate = new ArmRotate(map.getArmRotateMap());

    private Auto auto = new Auto(drive);

    @Autonomous(defaultAuto = true)
    public CommandBase exampleAuto = auto.exampleAuto();

    @Override
    public void teleopInit() {
    }

    @Override
    public void robotInit() {
        super.robotInit();

        Logger.getInstance().recordMetadata("ProjectName", "FRC-2023"); // Set a metadata value
        map.setupLogging();
        if (!isReal()) {
            setUseTiming(false); // Run as fast as possible
        }
        // Start logging! No more data receivers, replay sources, or metadata values may
        // be added.
        Logger.getInstance().start();
    }

    @Override
    public void configureButtonBindings() {
        // DRIVE CONTROLLER
        // Drive
        driveController.rightBumper().onTrue(drive.setSpeedCoef(0.25, 0.35)).onFalse(drive.setSpeedCoef(1, 1));
        driveController.a().onTrue(drive.driveToNearest());
        driveController.back().onTrue(runOnce(drive::resetGyro));

        // COPILOT CONTROLLER
        // Intake
        // copilotController.leftBumper().onTrue(intake.coneRelease());
        copilotController.a().onTrue(intake.coneGrab());
        copilotController.x().onTrue(intake.coneRelease());
        copilotController.b().onTrue(intake.grab());
        copilotController.rightBumper().whileTrue(arm.resetZero(() -> copilotController.getTriggers()));
        // copilotController.b().whileTrue(intake.cubeGrab());
        // copilotController.y().whileTrue(intake.cubeRelease());
        // Arm
        // extend and rotate are in default commands
        copilotController.start().onTrue(arm.zeroVelocityCheck());
        copilotController.back().onTrue(armRotate.zeroVelocityCheck());
        // will need buttons for the scoring positions
        copilotController.y().whileTrue(armRotate.moveTo(EnumLevel.HPS_PICKUP));

        // Led
        copilotController.povUp().whileTrue(led.setYellow());
        copilotController.povDown().whileTrue(led.setPurple());
    }

    @Override
    public void populateDashboard() {

    }

    @Override
    public void setDefaultCommands() {
        drive.setDefaultCommand(
                drive.drive(driveController::getLeftX, driveController::getLeftY, driveController::getRightX));

        led.setDefaultCommand(led.colorAlliance());
        arm.setDefaultCommand(arm.manual(copilotController::getTriggers));
        armRotate.setDefaultCommand(armRotate.move(RobotUtils.deadbandAxis(.1, () -> -copilotController.getLeftY())));

    }
}