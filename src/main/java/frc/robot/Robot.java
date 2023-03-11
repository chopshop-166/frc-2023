package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.Autonomous;
import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.maps.RobotMap;
// $Imports$
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmRotate;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Led;

public class Robot extends CommandRobot {

    private RobotMap map = getMapForName("FrostBite", RobotMap.class, "frc.robot.maps");
    private ButtonXboxController driveController = new ButtonXboxController(0);
    private ButtonXboxController copilotController = new ButtonXboxController(1);

    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();

    StringPublisher gamePiece = ntinst.getStringTopic("Game Piece").publish();
    StringSubscriber gamePieceSub = ntinst.getStringTopic("Game Piece").subscribe("Cone");

    // $Subsystems$
    private Arm arm = new Arm(map.getArmMap());

    Intake intake = new Intake(map.getIntakeMap());

    private Drive drive = new Drive(map.getDriveMap());
    private Led led = new Led(map.getLedMap());
    private ArmRotate armRotate = new ArmRotate(map.getArmRotateMap());

    private Auto auto = new Auto(drive, arm, armRotate, intake);

    @Autonomous(defaultAuto = true)
    public CommandBase noAuto = runOnce(() -> {
    }).withName("No Auto");

    @Autonomous
    public CommandBase simpleAuto = auto.oneSimpleConeTest();

    @Autonomous
    public CommandBase simpleTaxiAuto = auto.oneConeTaxiTest();

    @Autonomous
    public CommandBase simpleTaxiWireAuto = auto.oneConeTaxiWire();

    private CommandBase driveScoreHigh = sequence(
            armRotate.moveTo(EnumLevel.HIGH_SCORE), drive.driveToNearest(), arm.moveTo(EnumLevel.HIGH_SCORE),
            intake.coneRelease());

    private CommandBase driveScoreHighNode = sequence(
            armRotate.moveTo(EnumLevel.HIGH_SCORE), drive.driveToNearest(),
            new ConditionalCommand(arm.moveTo(EnumLevel.HIGH_SCORE), runOnce(() -> {
            }), () -> {
                return gamePieceSub.get() == "Cone";
            }));

    public CommandBase driveScoreMidNode = sequence(
            armRotate.moveTo(EnumLevel.MEDIUM_SCORE), drive.driveToNearest(),
            new ConditionalCommand(arm.moveTo(EnumLevel.MEDIUM_SCORE), runOnce(() -> {
            }), () -> {
                return gamePieceSub.get() == "Cone";
            }));

    public CommandBase scoreMidNode = sequence(
            armRotate.moveTo(EnumLevel.MEDIUM_SCORE), (arm.moveTo(EnumLevel.MEDIUM_SCORE)),
            (armRotate.moveTo(EnumLevel.MEDIUM_SCORE_ACTUAL)), (arm.moveTo(EnumLevel.ARM_STOWED)));

    public CommandBase scoreHighNode = sequence(
            armRotate.moveTo(EnumLevel.HIGH_SCORE), (arm.moveTo(EnumLevel.HIGH_SCORE)),
            (armRotate.moveTo(EnumLevel.HIGH_SCORE_ACTUAL)), (arm.moveTo(EnumLevel.ARM_STOWED)));

    public CommandBase grabCube() {
        return sequence(runOnce(() -> {
            gamePiece.set("Cube");
        }), led.setPurple());
    }

    public CommandBase grabCone() {
        return sequence(runOnce(() -> {
            gamePiece.set("Cone");
        }), led.setYellow());
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
    public void teleopInit() {
        super.teleopInit();
        armRotate.brakeMode().schedule();
    }

    @Override
    public void configureButtonBindings() {
        // DRIVE CONTROLLER
        // Drive
        driveController.rightBumper().onTrue(drive.setSpeedCoef(0.25, 0.35)).onFalse(drive.setSpeedCoef(1, 1));
        driveController.back().onTrue(drive.resetGyroCommand());

        // Arm

        // COPILOT CONTROLLER
        // Intake
        copilotController.a().onTrue(intake.grab());
        copilotController.b().onTrue(intake.toggle());
        copilotController.x().whileTrue(intake.cubeRelease());

        // Arm
        // extend and rotate are in default commands
        copilotController.start().onTrue(arm.zeroVelocityCheck());
        copilotController.back().whileTrue(armRotate.resetZero());

        // Automatic
        copilotController.rightBumper().onTrue(grabCube());
        copilotController.leftBumper().onTrue(grabCone());
        // will need buttons for the enums
        copilotController.y().whileTrue(armRotate.moveTo(EnumLevel.HPS_PICKUP));
        copilotController.povUp()
                .whileTrue(scoreHighNode);
        copilotController.povRight()
                .whileTrue(scoreMidNode);
        copilotController.povLeft()
                .whileTrue(arm.moveTo(EnumLevel.ARM_STOWED).andThen(armRotate.moveTo(EnumLevel.ARM_STOWED)));

        // Led

    }

    @Override
    public void populateDashboard() {
    }

    @Override
    public void setDefaultCommands() {
        drive.setDefaultCommand(
                drive.drive(driveController::getLeftX, driveController::getLeftY, driveController::getRightX));

        // led.setDefaultCommand(led.colorAlliance());
        arm.setDefaultCommand(arm.manual(copilotController::getTriggers));
        armRotate.setDefaultCommand(armRotate.move(() -> -copilotController.getLeftY()));
        led.setDefaultCommand(led.ColdFire());

    }
}