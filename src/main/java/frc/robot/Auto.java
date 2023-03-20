package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.race;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import com.chopshop166.chopshoplib.commands.FunctionalWaitCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.auto.AutoPath;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmRotate;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;

public class Auto {
    // Declare references to subsystems
    Drive drive;
    ArmExtend armExtend;
    ArmRotate armRotate;
    Intake intake;

    // Pass in all subsystems
    public Auto(Drive drive, ArmExtend armExtend, ArmRotate armRotate, Intake intake) {
        this.drive = drive;
        this.armExtend = armExtend;
        this.armRotate = armRotate;
        this.intake = intake;
    }

    private CommandBase backUp(double speed, double seconds) {
        return race(
                drive.driveRaw(() -> 0, () -> -speed, () -> 0),
                new FunctionalWaitCommand(() -> seconds)).andThen(drive.safeStateCmd());
    }

    private CommandBase moveLeft(double speed, double seconds) {
        return race(
                drive.driveRaw(() -> speed, () -> 0, () -> 0),
                new FunctionalWaitCommand(() -> seconds)).andThen(drive.safeStateCmd());
    }

    private CommandBase armScore(ArmPresets aboveLevel, ArmPresets scoreLevel) {
        return sequence(
                armRotate.moveTo(aboveLevel), armExtend.moveTo(aboveLevel),
                armRotate.moveTo(scoreLevel), armExtend.moveTo(ArmPresets.ARM_STOWED));
    }

    public CommandBase scoreCone(ArmPresets aboveLevel, ArmPresets scoreLevel) {
        return sequence(
                armRotate.moveTo(aboveLevel),
                race(drive.driveToNearest(), new FunctionalWaitCommand(() -> 2)),
                armScore(aboveLevel, scoreLevel));
    }

    public CommandBase scoreConeSimple() {
        return race(new FunctionalWaitCommand(() -> 8),
                sequence(
                        armRotate.moveTo(ArmPresets.HIGH_SCORE),
                        backUp(-1.0, 1.5),
                        armScore(ArmPresets.HIGH_SCORE, ArmPresets.HIGH_SCORE_ACTUAL)));

    }

    public CommandBase scoreConeSimpleSlow() {
        return race(new FunctionalWaitCommand(() -> 8),
                sequence(
                        armRotate.moveTo(ArmPresets.HIGH_SCORE),
                        backUp(-1.0, 0.3),
                        armScore(ArmPresets.HIGH_SCORE, ArmPresets.HIGH_SCORE_ACTUAL)));

    }

    //// End of some stuff that Joe wrote
    //

    public CommandBase prepareToScoreCone() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                armExtend.zeroVelocityCheck(),
                backUp(0.5, 0.5),
                armRotate.moveTo(ArmPresets.HIGH_SCORE));
    }

    public CommandBase scoreCone() {
        return sequence(
                armExtend.moveTo(ArmPresets.HIGH_SCORE),
                timingWait(),
                intake.coneRelease(),
                timingWait());
    }

    public CommandBase stowArmCloseIntake() {
        return sequence(
                armExtend.moveTo(ArmPresets.ARM_STOWED),
                intake.coneGrab(),
                armRotate.moveTo(ArmPresets.ARM_STOWED));
    }

    public CommandBase pickUpCube() {
        return sequence(
                armRotate.moveTo(ArmPresets.CUBE_PICKUP),
                intake.grab(),
                armExtend.moveTo(ArmPresets.CUBE_PICKUP));
    }

    public CommandBase scoreCube() {
        return sequence(
                armRotate.moveTo(ArmPresets.HIGH_SCORE),
                timingWait(),
                intake.cubeRelease(),
                timingWait());
    }

    // SEQUENCES TO JUST SCORE A CONE - from starting positions
    private CommandBase startGridScoreCone(AutoPath upToStation, AutoPath backedUp) {
        return sequence(
                prepareToScoreCone(),
                upToStation.getPath(drive),
                scoreCone(),
                backedUp.getPath(drive),
                stowArmCloseIntake());
    }

    private CommandBase pickupCubeN(AutoPath readyForPickup, AutoPath goToPickup) {
        return sequence(
                readyForPickup.getPath(drive),
                pickUpCube(),
                goToPickup.getPath(drive));
    }

    public CommandBase scoreConePos(int conePos) {
        switch (conePos) {
            case 1:
                return startGridScoreCone(AutoPath.UP_TO_CONE_STATION_1, AutoPath.BACKED_UP_1)
                        .withName("Score Cone (Pos 1)");
            case 2:
                return startGridScoreCone(AutoPath.UP_TO_CONE_STATION_2, AutoPath.BACKED_UP_2)
                        .withName("Score Cone (Pos 2)");
            case 3:
                return startGridScoreCone(AutoPath.UP_TO_CONE_STATION_3, AutoPath.BACKED_UP_3)
                        .withName("Score Cone (Pos 3)");
            case 4:
                return startGridScoreCone(AutoPath.UP_TO_CONE_STATION_4, AutoPath.BACKED_UP_4)
                        .withName("Score Cone (Pos 4)");
            case 5:
                return startGridScoreCone(AutoPath.UP_TO_CONE_STATION_5, AutoPath.BACKED_UP_5)
                        .withName("Score Cone (Pos 4)");
            case 6:
                return startGridScoreCone(AutoPath.UP_TO_CONE_STATION_6, AutoPath.BACKED_UP_6)
                        .withName("Score Cone (Pos 4)");
            default:
                return none().withName("None");
        }
    }

    public CommandBase pickupCubePos(int cubePos) {
        switch (cubePos) {
            case 7:
                return pickupCubeN(AutoPath.READY_FOR_PICKUP_7, AutoPath.GO_TO_PICKUP_7).withName("Pickup (Pos 7)");
            case 8:
                return pickupCubeN(AutoPath.READY_FOR_PICKUP_8, AutoPath.GO_TO_PICKUP_8).withName("Pickup (Pos 8)");
            case 9:
                return pickupCubeN(AutoPath.READY_FOR_PICKUP_9, AutoPath.GO_TO_PICKUP_9).withName("Pickup (Pos 9)");
            case 10:
                return pickupCubeN(AutoPath.READY_FOR_PICKUP_10, AutoPath.GO_TO_PICKUP_10).withName("Pickup (Pos 10)");
            default:
                return none().withName("None");
        }

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

    public CommandBase combinedAuto(int coneScore, int cubePos, int cubeScorePos) {
        CommandBase coneScoreCmd = scoreConePos(coneScore);
        CommandBase pickupCubeCmd = pickupCubePos(cubePos);
        CommandBase scoreCubeCmd = scoreCubePos(cubeScorePos);

        CommandBase outOfCommunity = none();
        CommandBase returnToCommunity = none();
        if (coneScore == 1 || coneScore == 2 || coneScore == 3) {
            outOfCommunity = AutoPath.OUT_OF_COMMUNITY_1_2_3.getPath(drive);
            returnToCommunity = AutoPath.INTO_COMMUNITY_1_2_3.getPath(drive);
        } else if (coneScore == 4 || coneScore == 5 || coneScore == 6) {
            outOfCommunity = AutoPath.OUT_OF_COMMUNITY_4_5_6.getPath(drive);
            returnToCommunity = AutoPath.INTO_COMMUNITY_4_5_6.getPath(drive);
        }

        return sequence(
                coneScoreCmd,
                outOfCommunity,
                pickupCubeCmd,
                returnToCommunity,
                scoreCubeCmd,
                scoreCube()

        ).withName(coneScoreCmd.getName() + " " + pickupCubeCmd.getName() + " " + scoreCubeCmd.getName());
    }

    // Score cone and back up onto charge station (from pos 1) and then balance
    public CommandBase scoreConeBalance() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                armExtend.zeroVelocityCheck(),
                backUp(0.5, 0.5),
                armRotate.moveTo(ArmPresets.HIGH_SCORE),
                AutoPath.UP_TO_CONE_STATION_1.getPath(drive),
                scoreCone(),
                AutoPath.BACKED_UP_1.getPath(drive),
                stowArmCloseIntake(),
                AutoPath.ON_CHARGE_STATION_14.getPath(drive)
        // will need values for this ^
        // add whatever balance command that we do

        )
                .withName("Score Cone Balance");
    }

    //// Start of some more stuff that Joe wrote

    public CommandBase oneConeTest() {
        return sequence(
                armExtend.zeroVelocityCheck(),
                scoreCone(ArmPresets.HIGH_SCORE, ArmPresets.HIGH_SCORE_ACTUAL),
                backUp(1.2, 0.5),
                armRotate.moveTo(ArmPresets.ARM_STOWED))
                .withName("One Cone Test");
    }

    public CommandBase oneSimpleConeTest() {
        return sequence(
                armExtend.zeroVelocityCheck(),
                scoreConeSimple(),
                backUp(0.5, 0.5),
                armRotate.moveTo(ArmPresets.ARM_STOWED))
                .withName("Simple One Cone Test");
    }

    public CommandBase oneConeTaxiTest() {
        return sequence(
                drive.setGyro180(),
                scoreConeSimple(),
                backUp(0.5, 0.5),
                race(new FunctionalWaitCommand(() -> 3),
                        armRotate.moveTo(ArmPresets.ARM_STOWED)),
                backUp(1.5, 3.5))
                .withName("(MAIN) One Cone Mobolity");
    }

    public CommandBase axisConeMobility() {
        return sequence(
                drive.setGyro180(),
                backUp(1, 0.3),
                scoreConeSimpleSlow(),
                backUp(1, 0.3),
                race(new FunctionalWaitCommand(() -> 3),
                        armRotate.moveTo(ArmPresets.ARM_STOWED)),
                backUp(1.5, 3.5))

                .withName("(TEST) One Cone Mobolity");
    }

    public CommandBase moveDistanceTest() {
        return sequence(
                drive.setGyro180(),
                scoreConeSimple(),
                backUp(0.5, 0.5),
                armRotate.moveTo(ArmPresets.ARM_STOWED),
                drive.driveDistance(3, 1, Rotation2d.fromDegrees(2)))
                .withName("Move Distance Auto");
    }

    public CommandBase preTest() {
        return sequence(
                AutoPath.PRE_TEST.getPath(drive)).withName("(TEST) Pre Test");
    }

    public CommandBase oneConeTaxiWire() {
        return sequence(
                drive.setGyro180(),
                scoreConeSimple(),
                backUp(0.5, 0.5),
                armRotate.moveTo(ArmPresets.ARM_STOWED),
                moveLeft(0.5, 0.25),
                backUp(1.2, 4)

        ).withName("Wire Guard One Cone Mobolity");
    }

    public CommandBase oneConeTaxiNoCable() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                armExtend.zeroVelocityCheck(),

                armRotate.moveTo(ArmPresets.HIGH_SCORE),
                AutoPath.UP_TO_CONE_STATION_1.getPath(drive),
                scoreCone(),
                AutoPath.BACKED_UP_1.getPath(drive),
                armExtend.moveTo(ArmPresets.ARM_STOWED),
                intake.coneGrab(),
                armRotate.moveTo(ArmPresets.ARM_STOWED),
                AutoPath.OUT_OF_COMMUNITY_1_2_3.getPath(drive)

        ).withName("One Cone Taxi No Cable");
    }

    //// End of some more stuff that Joe wrote

    private CommandBase timingWait() {
        return new FunctionalWaitCommand(() -> 0.25);
    }

}