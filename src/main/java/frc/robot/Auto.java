package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.race;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.util.Arrays;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.chopshop166.chopshoplib.commands.FunctionalWaitCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

    private static final double blueX = 1.8;
    private static final Rotation2d rotation0 = Rotation2d.fromDegrees(0);
    private static final Rotation2d rotation180 = Rotation2d.fromDegrees(180);

    public enum Path {

        UP_TO_CONE_STATION(0.05,
                new Pose2d(1.7539,
                        5.0123, Rotation2d.fromDegrees(180))),

        BACKED_UP(0.05,
                new Pose2d(2.1,
                        5.0123, Rotation2d.fromDegrees(180))),

        OUT_OF_COMMUNITY(0.2,
                new Pose2d(2.1877, 4.7327, Rotation2d.fromDegrees(180)),
                new Pose2d(5.1150, 4.7327, Rotation2d.fromDegrees(0))

        ),

        READY_FOR_PICKUP(0.2,
                new Pose2d()),

        ON_CHARGE_STATION(0.2,
                new Pose2d()),

        CONE_5(0.05,
                new Pose2d(blueX, 5.27, rotation0)),

        CONE_4(0.05,
                new Pose2d(blueX, 4.11, rotation0)),

        CONE_3(0.05,
                new Pose2d(blueX, 3.51, rotation0)),

        CONE_2(0.05,
                new Pose2d(blueX, 2.36, rotation0)),

        CONE_1(0.05,
                new Pose2d(blueX, 1.8, rotation0)),

        CONE_0(0.05,
                new Pose2d(blueX, 0.63, rotation0));

        Pose2d poses[];
        double tolerance;

        private Path(double tolerance, Pose2d... poses) {
            this.poses = poses;
            this.tolerance = tolerance;
        }

        // Create a sequence command to drive to each pose
        public CommandBase getPath(Drive drive) {

            return sequence(
                    Arrays.stream(poses).map((pos) -> drive.driveTo(pos,
                            this.tolerance)).toArray(CommandBase[]::new))
                    .withName(this.name()).andThen(drive.safeStateCmd());
        }
    }

    /**
     * 5.42 N
     * Next is 4.74 B
     * Then 4.15 N
     * Then 3.56 N
     * Then 2.95 B
     * Then 2.36 N
     * Then 1.82 N
     * Then 1.23 B
     * And the one closest to judge table is 0.6 N
     * The x values for red (where the robot will go) are 15.45
     * And blue is 5. 44
     */

    private CommandBase backUp(double speed, double seconds) {
        return sequence(race(
                drive.driveRaw(() -> 0, () -> -speed, () -> 0),
                new FunctionalWaitCommand(() -> seconds)), drive.safeStateCmd())

        ;
    }

    private CommandBase moveLeft(double speed, double seconds) {
        return sequence(race(
                drive.driveRaw(() -> speed, () -> 0, () -> 0),
                new FunctionalWaitCommand(() -> seconds)), drive.safeStateCmd())

        ;
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

    public CommandBase startGridScoreCone() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                armExtend.zeroVelocityCheck(),

                // armRotate.moveTo(EnumLevel.HIGH_SCORE),
                backUp(0.5, 0.5),
                armRotate.moveTo(ArmPresets.HIGH_SCORE),
                Path.UP_TO_CONE_STATION.getPath(drive),
                armExtend.moveTo(ArmPresets.HIGH_SCORE),
                new FunctionalWaitCommand(() -> 0.25),
                intake.coneRelease(),
                new FunctionalWaitCommand(() -> 0.25),
                Path.BACKED_UP.getPath(drive),
                armExtend.moveTo(ArmPresets.ARM_STOWED),
                intake.coneGrab(),
                armRotate.moveTo(ArmPresets.ARM_STOWED),
                Path.OUT_OF_COMMUNITY.getPath(drive)

        )
                .withName("Start Grid Score Cone");
    }

    // For position next to barrier - blue side
    // Score cone and pickup a cube
    public CommandBase scoreConePickupCube() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                armExtend.zeroVelocityCheck(),

                backUp(0.5, 0.5),
                armRotate.moveTo(ArmPresets.HIGH_SCORE),
                Path.UP_TO_CONE_STATION.getPath(drive),
                armExtend.moveTo(ArmPresets.HIGH_SCORE),
                new FunctionalWaitCommand(() -> 0.25),
                intake.coneRelease(),
                new FunctionalWaitCommand(() -> 0.25),
                Path.BACKED_UP.getPath(drive),
                armExtend.moveTo(ArmPresets.ARM_STOWED),
                intake.coneGrab(),
                armRotate.moveTo(ArmPresets.ARM_STOWED),
                Path.OUT_OF_COMMUNITY.getPath(drive),
                // rotate the robot here (180 degrees I would assume)
                // need to get values for \/
                Path.READY_FOR_PICKUP.getPath(drive),
                armRotate.moveTo(ArmPresets.CUBE_PICKUP),
                intake.grab(),
                armExtend.moveTo(ArmPresets.CUBE_PICKUP)

        )
                .withName("Score Cone Pickup Cube");
    }

    // Score cone and back up onto charge station (from mid position) and then
    // balance
    public CommandBase scoreConeBalance() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                armExtend.zeroVelocityCheck(),

                backUp(0.5, 0.5),
                armRotate.moveTo(ArmPresets.HIGH_SCORE),
                Path.UP_TO_CONE_STATION.getPath(drive),
                armExtend.moveTo(ArmPresets.HIGH_SCORE),
                new FunctionalWaitCommand(() -> 0.25),
                intake.coneRelease(),
                new FunctionalWaitCommand(() -> 0.25),
                Path.BACKED_UP.getPath(drive),
                armExtend.moveTo(ArmPresets.ARM_STOWED),
                intake.coneGrab(),
                armRotate.moveTo(ArmPresets.ARM_STOWED),
                Path.ON_CHARGE_STATION.getPath(drive)
        // will need values for this ^
        // add whatever balance command that we do

        )
                .withName("Score Cone Balance");
    }

    // For position next to barrier - blue side
    // Score cone, pickup and score a cube
    public CommandBase scoreConePickupScoreCube() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                armExtend.zeroVelocityCheck(),

                backUp(0.5, 0.5),
                armRotate.moveTo(ArmPresets.HIGH_SCORE),
                Path.UP_TO_CONE_STATION.getPath(drive),
                armExtend.moveTo(ArmPresets.HIGH_SCORE),
                new FunctionalWaitCommand(() -> 0.25),
                intake.coneRelease(),
                new FunctionalWaitCommand(() -> 0.25),
                Path.BACKED_UP.getPath(drive),
                armExtend.moveTo(ArmPresets.ARM_STOWED),
                intake.coneGrab(),
                armRotate.moveTo(ArmPresets.ARM_STOWED),
                Path.OUT_OF_COMMUNITY.getPath(drive),
                // rotate the robot here (180 degrees I would assume)
                // need to get values for \/
                Path.READY_FOR_PICKUP.getPath(drive),
                armRotate.moveTo(ArmPresets.CUBE_PICKUP),
                intake.grab(),
                armExtend.moveTo(ArmPresets.CUBE_PICKUP),
                // rotate the robot here (180 degrees again)
                // move right
                Path.BACKED_UP.getPath(drive),
                armRotate.moveTo(ArmPresets.HIGH_SCORE),
                new FunctionalWaitCommand(() -> 0.25),
                intake.cubeRelease(),
                new FunctionalWaitCommand(() -> 0.25)

        )
                .withName("Score Cone Pickup Score Cube");
    }

    // Need various auto sequences:
    // Score -> balance
    // Score -> out of community -> pickup
    // Score -> over charge stration -> pickup -> balance
    // Score -> pickup -> score

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

    public CommandBase mightBeDeadlyBalanceTest() {
        return sequence(
                armRotate.moveTo(ArmPresets.ARM_STOWED),
                drive.driveUntilTilted(),
                drive.balancePID()

        );
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

    public CommandBase oneConeTaxiWire() {
        return sequence(
                drive.setGyro180(),
                scoreConeSimple(),
                backUp(0.5, 0.5),
                armRotate.moveTo(ArmPresets.ARM_STOWED),
                moveLeft(0.5, 0.25),
                backUp(1.2, 4))
                .withName("Wire Guard One Cone Mobolity");
    }

    public CommandBase oneConeTaxiNoCable() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                armExtend.zeroVelocityCheck(),

                armRotate.moveTo(ArmPresets.HIGH_SCORE),
                Path.UP_TO_CONE_STATION.getPath(drive),
                armExtend.moveTo(ArmPresets.HIGH_SCORE),
                new FunctionalWaitCommand(() -> 0.25),
                intake.coneRelease(),
                new FunctionalWaitCommand(() -> 0.25),
                Path.BACKED_UP.getPath(drive),
                armExtend.moveTo(ArmPresets.ARM_STOWED),
                intake.coneGrab(),
                armRotate.moveTo(ArmPresets.ARM_STOWED),
                Path.OUT_OF_COMMUNITY.getPath(drive)

        )
                .withName("One Cone Taxi No Cable");
    }

}