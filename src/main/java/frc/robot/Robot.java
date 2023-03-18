package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.race;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.Autonomous;
import com.chopshop166.chopshoplib.RobotUtils;
import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.commands.FunctionalWaitCommand;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.maps.RobotMap;
// $Imports$
import frc.robot.subsystems.ArmExtend;
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
    private ArmExtend armExtend = new ArmExtend(map.getArmMap());

    Intake intake = new Intake(map.getIntakeMap());

    private Drive drive = new Drive(map.getDriveMap());
    private Led led = new Led(map.getLedMap());
    private ArmRotate armRotate = new ArmRotate(map.getArmRotateMap());

    private Auto auto = new Auto(drive, armExtend, armRotate, intake);

    @Autonomous(defaultAuto = true)
    public CommandBase noAuto = runOnce(() -> {
    }).withName("No Auto");

    @Autonomous
    public CommandBase simpleAuto = auto.oneSimpleConeTest();

    @Autonomous
    public CommandBase mobilityAuto = auto.axisConeMobility();

    @Autonomous
    public CommandBase simpleTaxiAuto = auto.oneConeTaxiTest();

    @Autonomous
    public CommandBase simpleTaxiWireAuto = auto.oneConeTaxiWire();

    private CommandBase driveScoreHigh = sequence(
            armRotate.moveTo(ArmPresets.HIGH_SCORE), drive.driveToNearest(), armExtend.moveTo(ArmPresets.HIGH_SCORE),
            intake.coneRelease());

    private CommandBase driveScoreHighNode = sequence(
            armRotate.moveTo(ArmPresets.HIGH_SCORE), drive.driveToNearest(),
            new ConditionalCommand(
                    armExtend.moveTo(ArmPresets.HIGH_SCORE).andThen(armRotate.moveTo(ArmPresets.HIGH_SCORE_ACTUAL))
                            .andThen(armExtend.moveTo(ArmPresets.ARM_STOWED)),
                    runOnce(() -> {
                    }), () -> {
                        return gamePieceSub.get() == "Cone";
                    }));

    public CommandBase driveScoreMidNode = sequence(
            armRotate.moveTo(ArmPresets.MEDIUM_SCORE), drive.driveToNearest(),
            new ConditionalCommand(
                    armExtend.moveTo(ArmPresets.MEDIUM_SCORE).andThen(armRotate.moveTo(ArmPresets.MEDIUM_SCORE_ACTUAL))
                            .andThen(armExtend.moveTo(ArmPresets.ARM_STOWED)),
                    runOnce(() -> {
                    }), () -> {
                        return gamePieceSub.get() == "Cone";
                    }));

    public CommandBase scoreMidNode = sequence(
            armRotate.moveTo(ArmPresets.MEDIUM_SCORE),
            new ConditionalCommand(
                    armExtend.moveTo(ArmPresets.MEDIUM_SCORE).andThen(armRotate.moveTo(ArmPresets.MEDIUM_SCORE_ACTUAL))
                            .andThen(armExtend.moveTo(ArmPresets.ARM_STOWED)),
                    runOnce(() -> {
                    }), () -> {
                        return gamePieceSub.get() == "Cone";
                    }));

    public CommandBase scoreHighNode = sequence(
            armRotate.moveTo(ArmPresets.HIGH_SCORE),
            new ConditionalCommand(
                    armExtend.moveTo(ArmPresets.HIGH_SCORE).andThen(armRotate.moveTo(ArmPresets.HIGH_SCORE_ACTUAL))
                            .andThen(armExtend.moveTo(ArmPresets.ARM_STOWED)),
                    runOnce(() -> {
                    }), () -> {
                        return gamePieceSub.get() == "Cone";
                    }));

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

    public CommandBase stowArm = sequence(
            intake.coneGrab(),
            new ConditionalCommand(
                    (armExtend.moveTo(ArmPresets.ARM_STOWED)), runOnce(() -> {
                    }), () -> {
                        return armExtend.data.distanceInches > 1;
                    }),
            (armExtend.zeroVelocityCheck()), (armRotate.moveTo(ArmPresets.ARM_STOWED)));

    public CommandBase pickUpGamePiece = sequence(
            new ConditionalCommand(
                    armRotate.moveTo(ArmPresets.CONE_PICKUP), armRotate.moveTo(ArmPresets.CUBE_PICKUP), () -> {
                        return gamePieceSub.get() == "Cone";
                    }));

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
        driveController.leftBumper().onTrue(drive.setSpeedCoef(0.4000000001, 0.5)).onFalse(drive.setSpeedCoef(1, 1));
        driveController.back().onTrue(drive.resetGyroCommand());

        // Arm

        // COPILOT CONTROLLER
        // Intake
        copilotController.a().onTrue(intake.grab().andThen(
                race(new FunctionalWaitCommand(() -> 2),
                        led.setGreen())));
        copilotController.b().onTrue(intake.toggle());
        copilotController.x().whileTrue(intake.cubeRelease());

        // Arm
        // extend and rotate are in default commands
        copilotController.start().onTrue(armExtend.zeroVelocityCheck());
        copilotController.back().whileTrue(armRotate.resetZero());

        // Automatic
        copilotController.rightBumper().onTrue(grabCone());
        copilotController.leftBumper().onTrue(grabCube());
        // will need buttons for the enums
        copilotController.y().whileTrue(armRotate.moveTo(ArmPresets.HPS_PICKUP));
        copilotController.povUp()
                .whileTrue(scoreHighNode);
        copilotController.povRight()
                .whileTrue(scoreMidNode);
        // stow arm when it is extended past 2 inches
        copilotController.povLeft()
                .whileTrue(stowArm);

        // copilotController.povDown()
        // .whileTrue(arm.moveTo(EnumLevel.CUBE_PICKUP).andThen(armRotate.moveTo(EnumLevel.CUBE_PICKUP)));

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
        armExtend.setDefaultCommand(armExtend.manual(copilotController::getTriggers));
        armRotate.setDefaultCommand(armRotate.move(RobotUtils.deadbandAxis(.1, () -> -copilotController.getLeftY())));
        led.setDefaultCommand(led.ColdFire());

    }
}