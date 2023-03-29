package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.race;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.chopshop166.chopshoplib.commands.FunctionalWaitCommand;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.auto.AutoPath;
import frc.robot.auto.ConeStation;
import frc.robot.auto.CubePickupLocation;
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
    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
    StringSubscriber gamePieceSub = ntinst.getStringTopic("Game Piece").subscribe("Cone");

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

    // THE ONE THAT ACTUALLY WORKS
    public CommandBase scoreConeSimpleSlow() {
        return race(new FunctionalWaitCommand(() -> 8),
                sequence(
                        drive.setGyro180(),
                        backUp(1.0, 0.4),
                        armRotate.moveTo(ArmPresets.HIGH_SCORE),
                        backUp(-1.0, 0.4),
                        armScore(ArmPresets.HIGH_SCORE, ArmPresets.HIGH_SCORE_ACTUAL),
                        backUp(1.0, 0.5),
                        stowArmCloseIntake()));

    }

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

    public CommandBase scoreHighNode() {
        return sequence(
                armRotate.moveTo(ArmPresets.HIGH_SCORE),
                new ConditionalCommand(
                        armExtend.moveTo(ArmPresets.HIGH_SCORE).andThen(armRotate.moveTo(ArmPresets.HIGH_SCORE_ACTUAL))
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
        // position to be in front of drive station in community? I'll add that
        // , drive.driveUntilTipped()
        // , drive.balance()

        ).withName(coneScoreCmd.getName() + " " + pickupCubeCmd.getName() + " " + scoreCubeCmd.getName());
    }

    // Score cone and back up onto charge station (from pos 1) and then balance
    public CommandBase scoreConeBalance() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                scoreConeSimpleSlow(),
                // backUp(0.3, 3.5),
                drive.driveUntilTipped(true),
                drive.balance()

        )
                .withName("Score Cone Balance");
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

    private CommandBase timingWait() {
        return new FunctionalWaitCommand(() -> 0.25);
    }

}