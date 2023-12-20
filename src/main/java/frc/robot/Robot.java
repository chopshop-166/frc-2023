package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.Autonomous;
import com.chopshop166.chopshoplib.RobotUtils;
import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.ConeStation;
import frc.robot.auto.CubePickupLocation;
import frc.robot.maps.Valkyrie;
import frc.robot.maps.RobotMap;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmRotate;
// $Imports$
import frc.robot.subsystems.BalanceArm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Led;
import frc.robot.Vision;

public class Robot extends CommandRobot {

    SendableChooser<ConeStation> conePosChooser = new SendableChooser<>();
    SendableChooser<CubePickupLocation> cubePosChooser = new SendableChooser<>();
    SendableChooser<Integer> cubeScorePosChooser = new SendableChooser<>();
    // private final SendableChooser<Command> autoChooser =
    // AutoBuilder.buildAutoChooser(); // Default auto will be
    // // `Commands.none()`;

    private RobotMap map = new Valkyrie();
    private ButtonXboxController driveController = new ButtonXboxController(0);
    private ButtonXboxController copilotController = new ButtonXboxController(1);

    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();

    StringPublisher gamePiece = ntinst.getStringTopic("Game Piece").publish();
    StringSubscriber gamePieceSub = ntinst.getStringTopic("Game Piece").subscribe("Cone");

    // $Subsystems$
    BalanceArm balanceArm = new BalanceArm(map.getBalanceArmMap());

    private ArmExtend armExtend = new ArmExtend(map.getArmMap());

    Intake intake = new Intake(map.getIntakeMap());

    private Drive drive = new Drive(map.getDriveMap());
    private Led led = new Led(map.getLedMap());
    private ArmRotate armRotate = new ArmRotate(map.getArmRotateMap());

    private Auto auto = new Auto(drive, armExtend, armRotate, intake, led, balanceArm);
    private Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

    @Autonomous(name = "No Auto")
    public Command noAuto = Commands.none();
    @Autonomous(name = "Score then balance")
    public Command scoreBalance = auto.scoreConeBalance();
    @Autonomous(name = "Just Score")
    public Command scorewhile = auto.scoreConeWhile(Commands.none());
    @Autonomous(name = "Score and pick up")
    public Command scoreAndPickUp = auto.leaveCommunityAndPickUP();

    // @Autonomous
    // public Command scoreLeaveBalance = auto.scoreConeLeaveAndBalance();

    @Autonomous(defaultAuto = true, name = "(MAIN) Score leave")
    public Command scoreThenLeave = auto.leaveCommunity();

    // @Autonomous(name = "pathplanner path 1")
    // public CommandBase pathOne = auto.fullAuto();

    // @Autonomous(name = "pathplanner path 1")
    // public CommandBase pathOne = auto.fullAuto();

    // @Autonomous(name = "Piecemeal Auto")
    // public Command buildCommand = new ProxyCommand(() -> {
    // ConeStation conePos = conePosChooser.getSelected();
    // CubePickupLocation cubePos = cubePosChooser.getSelected();
    // int cubeScorePos = cubeScorePosChooser.getSelected();
    // return auto.combinedAuto(conePos, cubePos, cubeScorePos);
    // });

    public Command driveScoreMidNode = sequence(
            armRotate.moveTo(ArmPresets.MEDIUM_SCORE),
            drive.driveToNearest(),
            armExtend.moveTo(ArmPresets.MEDIUM_SCORE),
            armRotate.moveTo(ArmPresets.MEDIUM_SCORE_DOWN),
            armExtend.moveTo(ArmPresets.ARM_STOWED));

    public Command scoreMidNode = sequence(
            armRotate.moveTo(ArmPresets.MEDIUM_SCORE, new Constraints(150,
                    1000)),
            armExtend.moveTo(ArmPresets.MEDIUM_SCORE),
            armRotate.moveTo(ArmPresets.MEDIUM_SCORE_DOWN, new Constraints(150,
                    100)),
            armExtend.moveTo(ArmPresets.ARM_STOWED));

    public Command scoreHighNode = sequence(
            armRotate.moveTo(ArmPresets.HIGH_SCORE, new Constraints(150,
                    1000)),
            armExtend.moveTo(ArmPresets.HIGH_SCORE),
            armRotate.moveTo(ArmPresets.HIGH_SCORE_DOWN, new Constraints(150,
                    100)),
            armExtend.moveTo(ArmPresets.ARM_STOWED));

    public Command grabCube() {
        return sequence(runOnce(() -> {
            gamePiece.set("Cube");
        }), led.setPurple());
    }

    public Command grabCone() {
        return sequence(runOnce(() -> {
            gamePiece.set("Cone");
        }), led.setYellow());
    }

    public Command rumbleOn() {
        return runOnce(() -> {
            copilotController.getHID().setRumble(RumbleType.kBothRumble, 1);
        });
    }

    public Command rumbleOff() {
        return runOnce(() -> {
            copilotController.getHID().setRumble(RumbleType.kBothRumble, 0);
        });
    }

    public Command rumbleAndIntakeSpinningOff() {
        return rumbleOff().andThen(led.colorAlliance());
    }

    public Command stowArm = sequence(
            led.setOrange(),
            intake.closeIntake(),
            armExtend.retract(0.4),
            armRotate.moveTo(ArmPresets.ARM_STOWED),
            led.colorAlliance());

    public Command pickUpGamePiece = sequence(
            new ConditionalCommand(
                    sequence(
                            armRotate.moveTo(ArmPresets.CONE_PICKUP), armExtend.moveTo(ArmPresets.CONE_PICKUP)),
                    sequence(
                            armRotate.moveTo(ArmPresets.CUBE_PICKUP), armExtend.moveTo(
                                    ArmPresets.CUBE_PICKUP),
                            intake.openIntake()),
                    () -> {
                        return gamePieceSub.get() == "Cone";
                    }));

    @Override
    public void robotInit() {
        super.robotInit();

        conePosChooser.setDefaultOption("Cone Pos 1", ConeStation.STATION_1);
        conePosChooser.addOption("Cone Pos 2", ConeStation.STATION_2);
        conePosChooser.addOption("Cone Pos 3", ConeStation.STATION_3);
        conePosChooser.addOption("Cone Pos 4", ConeStation.STATION_4);
        conePosChooser.addOption("Cone Pos 5", ConeStation.STATION_5);
        conePosChooser.addOption("Cone Pos 6", ConeStation.STATION_6);
        SmartDashboard.putData(conePosChooser);

        cubePosChooser.setDefaultOption("Pick Up Cube 1", CubePickupLocation.CUBE_1);
        cubePosChooser.addOption("Pick Up Cube 2", CubePickupLocation.CUBE_2);
        cubePosChooser.addOption("Pick Up Cube 3", CubePickupLocation.CUBE_3);
        cubePosChooser.addOption("Pick Up Cube 4", CubePickupLocation.CUBE_4);
        SmartDashboard.putData(cubePosChooser);

        cubeScorePosChooser.setDefaultOption("Don't Score Cube", 0);
        cubeScorePosChooser.addOption("Score Cube 11", 11);
        cubeScorePosChooser.addOption("Score Cube 12", 12);
        cubeScorePosChooser.addOption("Score Cube 13", 13);
        SmartDashboard.putData(cubeScorePosChooser);

        // SmartDashboard.putData("Auto Mode", autoChooser);

        Logger.recordMetadata("ProjectName", "FRC-2023"); // Set a metadata value
        map.setupLogging();
        if (!isReal()) {
            setUseTiming(false); // Run as fast as possible
        }
        // Start logging! No more data receivers, replay sources, or metadata values may
        // be added.
        Logger.start();
    }

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();
        Logger.recordOutput("Compressor/Pressure", compressor.getPressure());
        Logger.recordOutput("Compressor/PressureSwitch", compressor.getPressureSwitchValue());
        Logger.recordOutput("Compressor/Current", compressor.getCurrent());
    }

    public Command intakeGamePiece() {
        return rumbleOn().andThen(led.intakeSpinning(), intake.grab(), rumbleOff(),
                led.grabbedPiece()).finallyDo(intake::holdGamePiece);
    }

    @Override
    public void configureButtonBindings() {

        // DRIVE CONTROLLER
        // Drive
        driveController.rightBumper().onTrue(drive.setSpeedCoef(0.25, 0.35)).onFalse(drive.setSpeedCoef(1, 1));
        driveController.leftBumper().onTrue(drive.setSpeedCoef(0.4000000001, 0.5)).onFalse(drive.setSpeedCoef(1, 1));
        driveController.back().onTrue(drive.resetGyroCommand());
        driveController.rightStick().whileTrue(drive.robotCentricDrive(driveController::getLeftX,
                driveController::getLeftY, driveController::getRightX));

        // driveController.y().whileTrue(drive.balance());
        // driveController.x().whileTrue(drive.rotateToAngle(Rotation2d.fromDegrees(180),
        // () -> 0, () -> 0));
        // driveController.a().whileTrue(drive.rotateToAngle(Rotation2d.fromDegrees(0),
        // () -> 0, () -> 0));

        (new Trigger(() -> driveController.getRightTriggerAxis() > 0.5))
                .onTrue(balanceArm.pushDown()).onFalse(balanceArm.pushUp());

        driveController.a().whileTrue(auto.triangleAutoPathing());
        driveController.x().whileTrue(auto.barnAutoPathing());
        driveController.y().whileTrue(auto.knockoutAutoPathing());
        // driveController.b().onTrue(drive.setPose(new Pose2d(0.0, 0.0,
        // AutoConstants.ROTATION_0)));
        // driveController.b().onTrue(Vision.setPose(new Pose2d(0.0, 0.0, 0.0)));

        // Arm

        // COPILOT CONTROLLER
        // Intake
        copilotController.a()
                .onTrue(intakeGamePiece());
        copilotController.b().onTrue(intake.toggle().andThen(rumbleAndIntakeSpinningOff()));
        copilotController.x().onTrue(rumbleAndIntakeSpinningOff()).whileTrue(intake.cubeRelease());

        // Arm
        // extend and rotate are in default commands
        new Trigger(DriverStation::isEnabled).onTrue(armRotate.brakeMode());
        copilotController.start().onTrue(armRotate.toggleAbsolute());
        copilotController.back().whileTrue(armRotate.resetZero());

        // Automatic
        copilotController.rightBumper().onTrue(grabCone());
        copilotController.leftBumper().onTrue(grabCube());
        // will need buttons for the enums
        copilotController.y().whileTrue(armRotate.moveTo(ArmPresets.HPS_PICKUP, new Constraints(150,
                250)));
        copilotController.povUp()
                .whileTrue(scoreHighNode);
        copilotController.povRight()
                .whileTrue(scoreMidNode);
        // stow arm when it is extended past 2 inches
        copilotController.povLeft().or(copilotController.povUpLeft()).or(copilotController.povDownLeft())
                .whileTrue(stowArm)
                .onFalse(led.colorAlliance());
        copilotController.povDown()
                .whileTrue(pickUpGamePiece);

        // Led

    }

    @Override
    public void populateDashboard() {
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    // public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    // }

    @Override
    public void setDefaultCommands() {
        drive.setDefaultCommand(
                drive.drive(driveController::getLeftX, driveController::getLeftY, driveController::getRightX));

        // led.setDefaultCommand(led.colorAlliance());
        armExtend.setDefaultCommand(armExtend.manual(copilotController::getTriggers));
        armRotate.setDefaultCommand(armRotate.move(RobotUtils.deadbandAxis(.1, () -> -copilotController.getLeftY())));
    }
}