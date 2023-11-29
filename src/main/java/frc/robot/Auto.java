package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.race;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.chopshop166.chopshoplib.commands.FunctionalWaitCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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

public class Auto {
    // Declare references to subsystems
    Drive drive;
    ArmExtend armExtend;
    ArmRotate armRotate;
    Intake intake;
    BalanceArm balanceArm;
    Led led;
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

    private CommandBase moveFor(double speed, double seconds) {
        return race(
                drive.driveRaw(() -> 0, () -> -speed, () -> 0),
                new FunctionalWaitCommand(() -> seconds)).andThen(drive.safeStateCmd());
    }

    public CommandBase leaveCommunity() {
        return (scoreConeWhile(
                drive.driveRelative(new Translation2d(4, 0), 180, 6)));
    }

    public CommandBase leaveCommunityAndPickUP() {
        return scoreConeWhile(
                drive.driveRelative(new Translation2d(2, 0), 180, 3)).andThen(
                        drive.driveRelative(new Translation2d(3.25, 0.75), 270,
                                8))
                .andThen(pickUpCube());
    }

    private CommandBase armScore(ArmPresets aboveLevel, ArmPresets scoreLevel) {
        return sequence(
                armRotate.moveTo(aboveLevel).withTimeout(2), armExtend.moveTo(aboveLevel).withTimeout(
                        2),
                armRotate.moveTo(scoreLevel).withTimeout(2), armExtend.moveTo(ArmPresets.ARM_STOWED).withTimeout(2));
    }

    // THE ONE THAT ACTUALLY WORKS
    public CommandBase scoreConeWhile(CommandBase commandWhileStow) {
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
    public CommandBase scoreConeBalance() {
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

    public CommandBase scoreConeLeaveAndBalance() {
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

    public CommandBase prepareToScoreCone() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                armExtend.zeroVelocityCheck(),
                moveFor(0.5, 0.5),
                armRotate.moveTo(ArmPresets.HIGH_SCORE));
    }

    public CommandBase stowArmCloseIntake() {
        return sequence(
                armExtend.moveTo(ArmPresets.ARM_STOWED),
                intake.closeIntake(),
                armRotate.moveTo(ArmPresets.ARM_STOWED));
    }

    public CommandBase pickUpCube() {
        return sequence(
                armRotate.moveTo(ArmPresets.CUBE_PICKUP),
                // armExtend.moveTo(ArmPresets.CUBE_PICKUP),
                intake.openIntake(),
                intake.grab()

        // armExtend.retract(0.4),
        );
    }

    public CommandBase scoreCube() {
        return sequence(
                armRotate.moveTo(ArmPresets.HIGH_SCORE),
                timingWait(),
                intake.cubeRelease(),
                timingWait(),
                armRotate.moveTo(ArmPresets.ARM_STOWED));
    }

    public CommandBase scoreCubeLow() {
        return sequence(
                armRotate.moveTo(ArmPresets.CUBE_PICKUP),
                parallel(intake.cubeRelease(),
                        intake.closeIntake()),
                armRotate.moveTo(ArmPresets.ARM_STOWED));
    }

    public CommandBase scoreHighNode() {
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

    private CommandBase startGridScoreCone(AutoPath upToStation, AutoPath backedUp) {
        return sequence(
                prepareToScoreCone(),
                upToStation.getPath(drive),
                scoreHighNode(),
                backedUp.getPath(drive),
                stowArmCloseIntake());
    }

    private CommandBase pickupCubeN(AutoPath readyForPickup, AutoPath goToPickup) {
        return sequence(
                readyForPickup.getPath(drive),
                pickUpCube(),
                goToPickup.getPath(drive));
    }

    public CommandBase scoreCubePos(int cubeScorePos) {
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

    public CommandBase combinedAuto(ConeStation conePos, CubePickupLocation cubePos, int cubeScorePos) {
        CommandBase coneScoreCmd = startGridScoreCone(conePos.upToStation, conePos.backedUp)
                .withName("Score Cone (Pos " + conePos.number + ")");
        CommandBase pickupCubeCmd = pickupCubeN(cubePos.readyForPickup, cubePos.goToPickup)
                .withName("Pickup (Pos " + cubePos.number + ")");
        CommandBase scoreCubeCmd = scoreCubePos(cubeScorePos);

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

    private CommandBase timingWait() {
        return new FunctionalWaitCommand(() -> 0.25);
    }

    // Square auto using the drive relative command
    // Note: Tranlsation 2d x,y represents distance in the x plane, y plane
    public CommandBase squareAutoDriveRelative() {
        return sequence(
                drive.setGyro180(),
                drive.driveRelative(new Translation2d(0.0, 0.0), 0, 3),
                drive.driveRelative(new Translation2d(0.0, 2.0), 0, 3),
                drive.driveRelative(new Translation2d(-2.0, 2.0), 0, 3),
                drive.driveRelative(new Translation2d(-2.0, 0), 0, 3),
                drive.driveRelative(new Translation2d(0, 0), 0, 3)).withName("Square: ");
    }

    // Square auto using pathing
    public CommandBase squareAutoPathing() {
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
    public CommandBase knockoutAutoPathing() {
        return sequence(
                // Vision.setPose(Pose2d(0.0, 0.0, AutoConstants.ROTATION_0)),

                // drive.setPose(new Pose2d(0.0, 0.0,
                // AutoConstants.ROTATION_0)),

                AutoPath.KNOCKOUT_AUTO_POS1.getPath(drive),
                AutoPath.KNOCKOUT_AUTO_POS2.getPath(drive),
                parallel(AutoPath.KNOCKOUT_AUTO_POS3.getPath(drive),
                        pickUpCube()),
                AutoPath.KNOCKOUT_AUTO_POS4.getPath(drive),
                AutoPath.KNOCKOUT_AUTO_POS1.getPath(drive),
                AutoPath.KNOCKOUT_AUTO_POS5.getPath(drive),
                AutoPath.KNOCKOUT_AUTO_POS6.getPath(drive),
                scoreCubeLow(),
                AutoPath.KNOCKOUT_AUTO_POS1.getPath(drive)

        // AutoPath.LINE_AUTO_POS1.getPath(drive)
        );
    }

    // Triangle Auto
    public CommandBase triangleAutoPathing() {
        return sequence(
                AutoPath.TRIANGLE_AUTO_POS1.getPath(drive),
                // waitSeconds(1.5),
                AutoPath.TRIANGLE_AUTO_POS2.getPath(drive),
                // waitSeconds(1.5),
                AutoPath.TRIANGLE_AUTO_POS3.getPath(drive),
                // waitSeconds(1.5),
                AutoPath.TRIANGLE_AUTO_POS1.getPath(drive));

    }

    // Testing with a rectangle (Liam)
    public CommandBase rectangleAutoPathing() {
        return sequence(
                AutoPath.RECTANGLE_AUTO_POS1.getPath(drive),
                AutoPath.RECTANGLE_AUTO_POS2.getPath(drive),
                AutoPath.RECTANGLE_AUTO_POS3.getPath(drive),
                AutoPath.RECTANGLE_AUTO_POS4.getPath(drive));
    }

    // Square auto using driveRaw Command maybe?
    public CommandBase squareAutoMoveTurn() {
        return sequence(

        );
    }

    public void resetCoords() {
        // no clue how to do this
    }
}