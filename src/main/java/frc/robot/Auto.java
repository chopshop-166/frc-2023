package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.race;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.chopshop166.chopshoplib.commands.FunctionalWaitCommand;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;
// import com.pathplanner.lib.commands.FollowPathWithEvents;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;
// import com.pathplanner.lib.PathConstraints;

import org.ejml.equation.Sequence;

import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoPath;
import frc.robot.auto.ConeStation;
import frc.robot.auto.CubePickupLocation;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmRotate;
import frc.robot.subsystems.BalanceArm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Led;
import frc.robot.Vision;

public class Auto {
    // Declare references to subsystems
    Drive drive;
    ArmExtend armExtend;
    ArmRotate armRotate;
    Intake intake;
    BalanceArm balanceArm;
    Led led;
    Vision vision;
    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
    StringSubscriber gamePieceSub = ntinst.getStringTopic("Game Piece").subscribe("Cone");

    // Pass in all subsystems
    public Auto(Drive drive, ArmExtend armExtend, ArmRotate armRotate, Intake intake, Led led, BalanceArm balanceArm) {
        this.drive = drive;
        this.armExtend = armExtend;
        this.armRotate = armRotate;
        this.intake = intake;
        this.led = led;
        this.balanceArm = balanceArm;
    }

    private Command moveFor(double speed, double seconds) {
        return race(
                drive.driveRaw(() -> 0, () -> -speed, () -> 0),
                new FunctionalWaitCommand(() -> seconds)).andThen(drive.safeStateCmd());
    }

    public Command leaveCommunity() {
        return (scoreConeWhile(
                drive.driveRelative(new Translation2d(4, 0), 180, 6)));
    }

    public Command leaveCommunityAndPickUP() {
        return scoreConeWhile(
                drive.driveRelative(new Translation2d(2, 0), 180, 3)).andThen(
                        drive.driveRelative(new Translation2d(3.25, 0.75), 270,
                                8))
                .andThen(pickUpCube());
    }

    private Command armScore(ArmPresets aboveLevel, ArmPresets scoreLevel) {
        return sequence(
                armRotate.moveTo(aboveLevel).withTimeout(2), armExtend.moveTo(aboveLevel).withTimeout(
                        2),
                armRotate.moveTo(scoreLevel).withTimeout(2), armExtend.moveTo(ArmPresets.ARM_STOWED).withTimeout(2));
    }

    // THE ONE THAT ACTUALLY WORKS
    public Command scoreConeWhile(Command commandWhileStow) {
        return sequence(
                drive.setGyro180(),
                armExtend.zeroVelocityCheck(),
                // backUp(1.5, 0.2),
                armRotate.moveTo(ArmPresets.HIGH_SCORE).withTimeout(1.5),
                // backUp(-1.5, 0.2),
                drive.driveRelative(new Translation2d(-Units.inchesToMeters(4), 0), 180, 2),
                armScore(ArmPresets.HIGH_SCORE, ArmPresets.HIGH_SCORE_DOWN),
                moveFor(1.0, 0.3),
                parallel(stowArmCloseIntake(),
                        commandWhileStow));

    }

    // Score cone and back up onto charge station (from pos 1) and then balance
    public Command scoreConeBalance() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                scoreConeWhile(drive.driveUntilTipped(true).withTimeout(2)),
                led.balancing(),
                drive.balance().withTimeout(5),
                led.starPower(),
                balanceArm.pushUp()

        )
                .withName("Score Cone Balance");
    }

    public Command scoreConeLeaveAndBalance() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                scoreConeWhile(drive.driveUntilTipped(true)),
                drive.driveUntilNotTipped(true).withTimeout(1.0),
                waitSeconds(0.5),
                moveFor(1.0, 3),
                waitSeconds(2),
                drive.driveUntilTipped(false),
                led.balancing(),
                drive.balance(),
                led.starPower()

        )
                .withName("Score Cone, Leave, and Balance");
    }

    public Command prepareToScoreCone() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                armExtend.zeroVelocityCheck(),
                moveFor(0.5, 0.5),
                armRotate.moveTo(ArmPresets.HIGH_SCORE));
    }

    public Command stowArmCloseIntake() {
        return sequence(
                armExtend.moveTo(ArmPresets.ARM_STOWED),
                intake.closeIntake(),
                armRotate.moveTo(ArmPresets.ARM_STOWED));
    }

    public Command pickUpCube() {
        return sequence(
                armRotate.moveTo(ArmPresets.CUBE_PICKUP),
                // armExtend.moveTo(ArmPresets.CUBE_PICKUP),
                intake.openIntake(),
                intake.grab()

        // armExtend.retract(0.4),
        );
    }

    public Command scoreCube() {
        return sequence(
                armRotate.moveTo(ArmPresets.HIGH_SCORE),
                timingWait(),
                intake.cubeRelease(),
                timingWait(),
                armRotate.moveTo(ArmPresets.ARM_STOWED));
    }

    public Command scoreCubeLow() {
        return sequence(
                armRotate.moveTo(ArmPresets.CUBE_PICKUP),
                parallel(intake.cubeRelease(),
                        intake.closeIntake()),
                armRotate.moveTo(ArmPresets.ARM_STOWED));
    }

    public Command scoreHighNode() {
        return sequence(
                armRotate.moveTo(ArmPresets.HIGH_SCORE),
                new ConditionalCommand(
                        armExtend.moveTo(ArmPresets.HIGH_SCORE).andThen(armRotate.moveTo(ArmPresets.HIGH_SCORE_DOWN))
                                .andThen(armExtend.moveTo(ArmPresets.ARM_STOWED)),
                        runOnce(() -> {
                        }), () -> {
                            return gamePieceSub.get() == "Cone";
                        }));
    }

    private Command startGridScoreCone(AutoPath upToStation, AutoPath backedUp) {
        return sequence(
                prepareToScoreCone(),
                upToStation.getPath(drive),
                scoreHighNode(),
                backedUp.getPath(drive),
                stowArmCloseIntake());
    }

    private Command pickupCubeN(AutoPath readyForPickup, AutoPath goToPickup) {
        return sequence(
                readyForPickup.getPath(drive),
                pickUpCube(),
                goToPickup.getPath(drive));
    }

    public Command scoreCubePos(int cubeScorePos) {
        switch (cubeScorePos) {
            case 11:
                return AutoPath.CUBE_SCORE_11.getPath(drive).withName("Score Cube (Pos 11)");
            case 12:
                return AutoPath.CUBE_SCORE_12.getPath(drive).withName("Score Cube (Pos 12)");
            case 13:
                return AutoPath.CUBE_SCORE_13.getPath(drive).withName("Score Cube (Pos 13)");
            default:
                return none().withName("None");
        }
    }

    public Command combinedAuto(ConeStation conePos, CubePickupLocation cubePos, int cubeScorePos) {
        Command coneScoreCmd = startGridScoreCone(conePos.upToStation, conePos.backedUp)
                .withName("Score Cone (Pos " + conePos.number + ")");
        Command pickupCubeCmd = pickupCubeN(cubePos.readyForPickup, cubePos.goToPickup)
                .withName("Pickup (Pos " + cubePos.number + ")");
        Command scoreCubeCmd = scoreCubePos(cubeScorePos);

        return sequence(
                coneScoreCmd,
                conePos.communityPosition.outOfCommunity.getPath(drive),
                pickupCubeCmd,
                conePos.communityPosition.inCommunity.getPath(drive),
                scoreCubeCmd,
                scoreCube()
        // position to be in front of drive station in community? I'll add that <- ha
        // you thought, no because this auto is useless
        // , drive.driveUntilTipped()
        // , drive.balance()

        ).withName(coneScoreCmd.getName() + " " + pickupCubeCmd.getName() + " " + scoreCubeCmd.getName());
    }

    private Command timingWait() {
        return new FunctionalWaitCommand(() -> 0.25);
    }

    // Square auto using the drive relative command
    // Note: Tranlsation 2d x,y represents distance in the x plane, y plane
    public Command squareAutoDriveRelative() {
        return sequence(
                drive.setGyro180(),
                drive.driveRelative(new Translation2d(0.0, 0.0), 0, 3),
                drive.driveRelative(new Translation2d(0.0, 2.0), 0, 3),
                drive.driveRelative(new Translation2d(-2.0, 2.0), 0, 3),
                drive.driveRelative(new Translation2d(-2.0, 0), 0, 3),
                drive.driveRelative(new Translation2d(0, 0), 0, 3)).withName("Square: ");
    }

    // Square auto using pathing
    public Command squareAutoPathing() {
        return sequence(
                AutoPath.SQUARE_AUTO_POS1.getPath(drive),
                // waitSeconds(1.5),
                AutoPath.SQUARE_AUTO_POS2.getPath(drive),
                // waitSeconds(1.5),
                AutoPath.SQUARE_AUTO_POS3.getPath(drive),
                // waitSeconds(1.5),
                AutoPath.SQUARE_AUTO_POS4.getPath(drive),
                // waitSeconds(1.5),
                AutoPath.SQUARE_AUTO_POS1.getPath(drive)).withName("Square: (Pathing)");
    }

    // Line Auto
    public Command knockoutAutoPathing() {
        return sequence(
                // Vision.setPose(Pose2d(0.0, 0.0, AutoConstants.ROTATION_0)),

                // drive.setPose(new Pose2d(0.0, 0.0,
                // AutoConstants.ROTATION_0)),

                AutoPath.KNOCKOUT_AUTO_POS1.getPath(drive),
                AutoPath.KNOCKOUT_AUTO_POS2.getPath(drive),
                AutoPath.KNOCKOUT_AUTO_POS3.getPath(drive),
                AutoPath.KNOCKOUT_AUTO_POS4.getPath(drive),
                AutoPath.KNOCKOUT_AUTO_POS1.getPath(drive),
                AutoPath.KNOCKOUT_AUTO_POS5.getPath(drive),
                AutoPath.KNOCKOUT_AUTO_POS6.getPath(drive),
                AutoPath.KNOCKOUT_AUTO_POS1.getPath(drive)

        // AutoPath.LINE_AUTO_POS1.getPath(drive)
        );
    }

    // Triangle Auto
    public Command triangleAutoPathing() {
        return sequence(
                AutoPath.TRIANGLE_AUTO_POS1.getPath(drive),
                // waitSeconds(1.5),
                AutoPath.TRIANGLE_AUTO_POS2.getPath(drive),
                // waitSeconds(1.5),
                AutoPath.TRIANGLE_AUTO_POS3.getPath(drive),
                // waitSeconds(1.5),
                AutoPath.TRIANGLE_AUTO_POS1.getPath(drive));
    }

    public Command rectangleAutoPathing() {
        return sequence(
                AutoPath.RECTANGLE_AUTO_POS1.getPath(drive),
                AutoPath.RECTANGLE_AUTO_POS2.getPath(drive),
                AutoPath.RECTANGLE_AUTO_POS3.getPath(drive),
                AutoPath.RECTANGLE_AUTO_POS4.getPath(drive),
                AutoPath.RECTANGLE_AUTO_POS1.getPath(drive));
    }

    public Command barnAutoPathing() {
        return sequence(
                AutoPath.BARN_AUTO_BOTTOM_LEFT_CORNER.getPath(drive),
                AutoPath.BARN_AUTO_TOP_LEFT_CORNER.getPath(drive),
                AutoPath.BARN_AUTO_TIP.getPath(drive),
                AutoPath.BARN_AUTO_TOP_RIGHT_CORNER.getPath(drive),
                AutoPath.BARN_AUTO_TOP_LEFT_CORNER.getPath(drive),
                AutoPath.BARN_AUTO_BOTTOM_RIGHT_CORNER.getPath(drive),
                AutoPath.BARN_AUTO_TOP_RIGHT_CORNER.getPath(drive),
                AutoPath.BARN_AUTO_BOTTOM_LEFT_CORNER.getPath(drive),
                AutoPath.BARN_AUTO_BOTTOM_RIGHT_CORNER.getPath(drive),
                AutoPath.BARN_AUTO_BOTTOM_LEFT_CORNER.getPath(drive));
    }

    // Square auto using driveRaw Command maybe?
    public Command squareAutoMoveTurn() {
        return sequence(

        );
    }

    //// PATHPLANNER 2023 JUNK

    // public void pathPlannerAuto() {
    // List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("FullAuto",
    // new PathConstraints(4, 3));
    // HashMap<String, Command> eventMap = new HashMap<>();
    // eventMap.put("prepare to score", prepareToScoreCone());
    // eventMap.put("Score gamePiece", scoreHighNode());
    // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    // drive::getPose, // Pose2d supplier
    // drive::resetPose, // Pose2d consumer, used to reset odometry at the beginning
    // of auto
    // drive.kinematics, // SwerveDriveKinematics
    // new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation
    // error (used to create the X
    // // and Y PID controllers)
    // new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation
    // error (used to create the
    // // rotation controller)
    // drive::setModuleStates, // Module states consumer used to output to the drive
    // subsystem
    // eventMap,
    // true, // Should the path be automatically mirrored depending on alliance
    // color.
    // // Optional, defaults to true
    // drive // The drive subsystem. Used to properly set the requirements of path
    // following
    // // commands
    // );

    // Command fullAuto = autoBuilder.fullAuto(pathGroup);
    // // This will load the file "Example Path.path" and generate it with a max
    // // velocity of 4 m/s and a max acceleration of 3 m/s^2
    // PathPlannerTrajectory twoPiece = PathPlanner.loadPath("2 piece", new
    // PathConstraints(4, 3));

    // // This is just an example event map. It would be better to have a constant,
    // // global event map
    // // in your code that will be used by all path following commands.

    // FollowPathWithEvents multiPiece = new FollowPathWithEvents(
    // autoBuilder.fullAuto(twoPiece),
    // twoPiece.getMarkers(),
    // eventMap);
    // }

    // // Assuming this method is part of a drivetrain subsystem that provides the
    // // necessary methods
    // public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean
    // isFirstPath) {
    // return new SequentialCommandGroup(
    // new InstantCommand(() -> {
    // // Reset odometry for the first path you run during auto
    // if (isFirstPath) {
    // // this.resetOdometry(traj.getInitialHolonomicPose()); <--- Need to put in
    // and
    // // figure out how to do
    // vision.setPose(new Pose2d(0, 0, AutoConstants.ROTATION_0));
    // }
    // }),
    // new PPSwerveControllerCommand(
    // traj,
    // drive::getPose, // Pose supplier
    // drive.kinematics, // SwerveDriveKinematics
    // new PIDController(0.01, 0.00001, 0.0), // X controller. Tune these values for
    // your robot.
    // // Leaving them 0
    // // will only use feedforwards.
    // new PIDController(0.01, 0.00001, 0.0), // Y controller (usually the same
    // values as X controller)
    // new PIDController(0.01, 0.00001, 0.0), // Rotation controller. Tune these
    // values for your robot.
    // // Leaving
    // // them 0 will only use feedforwards.
    // drive::setModuleStates, // Module states consumer
    // true, // Should the path be automatically mirrored depending on alliance
    // color.
    // // Optional, defaults to true
    // drive // Requires this drive subsystem
    // ));
    // }

    // private void configureAutoCommands() {

    // // build auto path commands
    // List<PathPlannerTrajectory> auto1Paths = PathPlanner.loadPathGroup(
    // "testpath1",
    // AutoConstants.MAX_VELOCITY_METERS_PER_SECOND,
    // AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    // // Command autoTest = new SequentialCommandGroup(
    // // new FollowPathWithEvents(

    // // auto1Paths.get(0).getMarkers();
    // // ));

    // }

    public void resetCoords() {
        // no clue how to do this
    }

}