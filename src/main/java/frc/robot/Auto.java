package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.race;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.util.Arrays;

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

    // private static final double blueX = 1.8;
    private static final Rotation2d rotation0 = Rotation2d.fromDegrees(0);
    private static final Rotation2d rotation180 = Rotation2d.fromDegrees(180);

    private static final double uptoconex = 2.00;
    private static final double backedupx = 2.25;
    private static final double outofcommunityx = 6.00;

    ////////////////// TABLE OF CONTENTS \\\\\\\\\\\\\\\\\\
    ////// POSITIONS AND PATHS-------------------- lines 62 - 185
    //// Cone scoring sub-positions -------------- lines 86 - 124
    //// Cube scoring sub-positions -------------- lines 125 - 134
    //// Into and out of community --------------- lines 135 - 152
    //// Pickup positions ------------------------ lines 153 - 181
    //// Chargestation position(s?) -------------- lines 182 - 185
    ////// JOE CODE ------------------------------ lines 186 - 271
    ////// SEQUENCES ----------------------------- lines 272 - 1118
    //// Sequences to make stuff easier ---------- lines 272 - 311
    //// Just score a cone and exit community ---- lines 312 - 396
    //// Score, leave, pickup cube --------------- lines 397 - 615
    //// Score, leave, pickup, go back, score ---- lines 616 - 1099
    // Start 1-3, pickup 7-8, score 11-12 -------- lines 616 - 858
    // Start 4-6, pickup 9-10, score 12-13 ------- lines 859 - 1099
    //// Score cone and balance ------------------ lines 1100 - 1118
    ////// JOE CODE ------------------------------ lines 1119 - 1211

    // PLEASE update with every change ^

    public enum Path {

        ////// ALL POSITIONS \\\\\\
        // Cone scoring positions (also starting positions)
        //// Position 1 -> closest to barrier
        //// Position 2 -> second closest to barrier
        //// Position 3 -> middle / barrier side
        //// Position 4 -> middle / judging side
        //// Position 5 -> second closest to judging table
        //// Position 6 -> closest to judging table (gotta go over wire)

        // Pickup positions
        // Position 7 -> Closest to barrier
        // Pos. 8 -> middle - barrier side
        // Pos. 9 -> middle - judge table side
        // Pos. 10 -> closest to judging table

        // Cube scoring positions
        // Pos. 11 -> closest to barrier
        // Pos. 12 -> middle
        // Pos. 13 -> closest to judging table

        // Chargestation position
        // Pos. 14 -> direct center of chargestation

        // CONE STATION SUB-POSITIONS
        UP_TO_CONE_STATION_1(0.05,
                new Pose2d(uptoconex, 5.32, rotation180)),

        BACKED_UP_1(0.05,
                new Pose2d(backedupx, 5.32, rotation180)),

        UP_TO_CONE_STATION_2(0.05,
                new Pose2d(uptoconex, 4.1, rotation180)),

        BACKED_UP_2(0.05,
                new Pose2d(backedupx, 4.15, rotation180)),

        UP_TO_CONE_STATION_3(0.05,
                new Pose2d(uptoconex, 3.52, rotation180)),

        BACKED_UP_3(0.05,
                new Pose2d(backedupx, 3.55, rotation180)),

        UP_TO_CONE_STATION_4(0.05,
                new Pose2d(uptoconex, 2.37, rotation180)),

        BACKED_UP_4(0.05,
                new Pose2d(backedupx, 2.39, rotation180)),

        UP_TO_CONE_STATION_5(0.05,
                new Pose2d(uptoconex, 1.79, rotation180)),

        BACKED_UP_5(0.05,
                new Pose2d(backedupx, 1.81, rotation180)),

        UP_TO_CONE_STATION_6(0.05,
                new Pose2d(uptoconex, 0.61, rotation180)),

        BACKED_UP_6(0.05,
                new Pose2d(backedupx, 0.63, rotation180)),

        //
        // CUBE STATION POSITIONS
        CUBE_SCORE_11(0.05,
                new Pose2d(backedupx, 4.71, rotation180)),

        CUBE_SCORE_12(0.05,
                new Pose2d(backedupx, 2.96, rotation180)),

        CUBE_SCORE_13(0.05,
                new Pose2d(backedupx, 1.21, rotation180)),

        // INTO AND OUT OF COMMUNITY
        OUT_OF_COMMUNITY_1_2_3(0.2,
                new Pose2d(2.0, 5.30, rotation180),
                new Pose2d(outofcommunityx, 5.30, rotation180)),

        OUT_OF_COMMUNITY_4_5_6(0.2,
                new Pose2d(2.0, 0.63, rotation180),
                new Pose2d(outofcommunityx, 0.63, rotation180)),

        INTO_COMMUNITY_1_2_3(0.2,
                new Pose2d(outofcommunityx, 5.30, rotation180),
                new Pose2d(2.0, 5.30, rotation180)),

        INTO_COMMUNITY_4_5_6(0.2,
                new Pose2d(outofcommunityx, 0.63, rotation180),
                new Pose2d(2.0, 0.63, rotation180)),

        //
        // PICKUP POSITIONS
        READY_FOR_PICKUP_7(0.2,
                new Pose2d(outofcommunityx, 5.30, rotation0),
                new Pose2d(outofcommunityx, 4.92, rotation0)),

        GO_TO_PICKUP_7(0.2,
                new Pose2d(6.90, 4.92, rotation0)),

        READY_FOR_PICKUP_8(0.2,
                new Pose2d(outofcommunityx, 5.30, rotation0),
                new Pose2d(outofcommunityx, 3.64, rotation0)),

        GO_TO_PICKUP_8(0.2,
                new Pose2d(6.90, 3.64, rotation0)),

        READY_FOR_PICKUP_9(0.2,
                new Pose2d(outofcommunityx, 0.63, rotation0),
                new Pose2d(outofcommunityx, 2.35, rotation0)),

        GO_TO_PICKUP_9(0.2,
                new Pose2d(6.90, 2.35, rotation0)),

        READY_FOR_PICKUP_10(0.2,
                new Pose2d(outofcommunityx, 0.63, rotation0),
                new Pose2d(outofcommunityx, 1.09, rotation0)),

        GO_TO_PICKUP_10(0.2,
                new Pose2d(6.90, 1.09, rotation0)),

        // CHARGE STATION POSTION(S?)
        ON_CHARGE_STATION_14(0.2,
                new Pose2d()),

        //// Start of some stuff that Joe wrote
        PRE_TEST(0.05,
                new Pose2d(
                        14.165401626124469, 2.879876924969839, Rotation2d.fromRadians(-0.22656457143950046)));

        // Hey Joe I don't think we need this anymore
        /*
         * CONE_5(0.05, new Pose2d(blueX, 5.27, rotation0)),
         * 
         * CONE_4(0.05, new Pose2d(blueX, 4.11, rotation0)),
         * 
         * CONE_3(0.05, new Pose2d(blueX, 3.51, rotation0)),
         * 
         * CONE_2(0.05, new Pose2d(blueX, 2.36, rotation0)),
         * 
         * CONE_1(0.05, new Pose2d(blueX, 1.8, rotation0)),
         * 
         * CONE_0(0.05, new Pose2d(blueX, 0.63, rotation0));
         */

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
                new FunctionalWaitCommand(() -> 0.25),
                intake.coneRelease(),
                new FunctionalWaitCommand(() -> 0.25));
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
                new FunctionalWaitCommand(() -> 0.25),
                intake.cubeRelease(),
                new FunctionalWaitCommand(() -> 0.25));
    }

    // SEQUENCES TO JUST SCORE A CONE - from starting positions
    public CommandBase startGridScoreCone_1() {
        return sequence(
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_1.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_1.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_1_2_3.getPath(drive)

        )
                .withName("Start Grid Score Cone Pos 1");
    }

    public CommandBase startGridScoreCone_2() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_2.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_2.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_1_2_3.getPath(drive)

        )
                .withName("Start Grid Score Cone Pos 2");
    }

    public CommandBase startGridScoreCone_3() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_3.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_3.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_1_2_3.getPath(drive)

        )
                .withName("Start Grid Score Cone Pos 3");
    }

    public CommandBase startGridScoreCone_4() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_4.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_4.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_4_5_6.getPath(drive)

        )
                .withName("Start Grid Score Cone Pos 4");
    }

    public CommandBase startGridScoreCone_5() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_5.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_5.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_4_5_6.getPath(drive)

        )
                .withName("Start Grid Score Cone Pos 5");
    }

    public CommandBase startGridScoreCone_6() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_6.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_6.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_4_5_6.getPath(drive)

        )
                .withName("Start Grid Score Cone Pos 6");
    }

    //
    //
    // SCORE CONE, EXIT COMMUNITY, AND PICKUP A CUBE
    // Score cone and pickup a cube starting from pos. 1
    public CommandBase scoreCone_1_PickupCube_7() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_1.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_1.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_1_2_3.getPath(drive),
                Path.READY_FOR_PICKUP_7.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_7.getPath(drive)

        )
                .withName("Score Cone (Pos 1) Pickup Cube (Pos 7)");
    }

    // Score cone and pickup a cube starting from pos. 2
    public CommandBase scoreCone_2_PickupCube_7() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_2.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_2.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_1_2_3.getPath(drive),
                Path.READY_FOR_PICKUP_7.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_7.getPath(drive)

        )
                .withName("Score Cone (Pos 2) Pickup Cube (Pos 7)");
    }

    // Score cone and pickup a cube starting from pos. 3
    public CommandBase scoreCone_3_PickupCube_7() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_3.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_3.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_1_2_3.getPath(drive),
                Path.READY_FOR_PICKUP_7.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_7.getPath(drive)

        )
                .withName("Score Cone (Pos 3) Pickup Cube (Pos 7)");
    }

    // Score cone and pickup a cube starting from pos. 1
    public CommandBase scoreCone_1_PickupCube_8() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_1.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_1.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_1_2_3.getPath(drive),
                Path.READY_FOR_PICKUP_8.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_8.getPath(drive)

        )
                .withName("Score Cone (Pos 1) Pickup Cube (Pos 8)");
    }

    // Score cone and pickup a cube starting from pos. 2
    public CommandBase scoreCone_2_PickupCube_8() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_2.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_2.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_1_2_3.getPath(drive),
                Path.READY_FOR_PICKUP_8.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_8.getPath(drive)

        )
                .withName("Score Cone (Pos 2) Pickup Cube (Pos 8)");
    }

    // Score cone and pickup a cube starting from pos. 3
    public CommandBase scoreCone_3_PickupCube_8() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_3.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_3.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_1_2_3.getPath(drive),
                Path.READY_FOR_PICKUP_8.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_8.getPath(drive)

        )
                .withName("Score Cone (Pos 3) Pickup Cube (Pos 8)");
    }

    // Score cone and pickup a cube starting from pos. 4
    public CommandBase scoreCone_4_PickupCube_9() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_4.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_4.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_4_5_6.getPath(drive),
                Path.READY_FOR_PICKUP_9.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_9.getPath(drive)

        )
                .withName("Score Cone (Pos 4) Pickup Cube (Pos 9)");
    }

    // Score cone and pickup a cube starting from pos. 5
    public CommandBase scoreCone_5_PickupCube_9() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_5.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_5.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_4_5_6.getPath(drive),
                Path.READY_FOR_PICKUP_9.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_9.getPath(drive)

        )
                .withName("Score Cone (Pos 5) Pickup Cube (Pos 9)");
    }

    // Score cone and pickup a cube starting from pos. 6
    public CommandBase scoreCone_6_PickupCube_9() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_6.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_6.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_4_5_6.getPath(drive),
                Path.READY_FOR_PICKUP_9.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_9.getPath(drive)

        )
                .withName("Score Cone (Pos 4) Pickup Cube (Pos 9)");
    }

    // Score cone and pickup a cube starting from pos. 4
    public CommandBase scoreCone_4_PickupCube_10() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_4.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_4.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_4_5_6.getPath(drive),
                Path.READY_FOR_PICKUP_10.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_10.getPath(drive)

        )
                .withName("Score Cone (Pos 4) Pickup Cube (Pos 10)");
    }

    // Score cone and pickup a cube starting from pos. 5
    public CommandBase scoreCone_5_PickupCube_10() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_5.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_5.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_4_5_6.getPath(drive),
                Path.READY_FOR_PICKUP_10.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_10.getPath(drive)

        )
                .withName("Score Cone (Pos 5) Pickup Cube (Pos 10)");
    }

    // Score cone and pickup a cube starting from pos. 6
    public CommandBase scoreCone_6_PickupCube_10() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_6.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_6.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_4_5_6.getPath(drive),
                Path.READY_FOR_PICKUP_10.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_10.getPath(drive)

        )
                .withName("Score Cone (Pos 4) Pickup Cube (Pos 10)");
    }

    //
    //
    // SCORE CONE, EXIT COMMUN., PICKUP CUBE AND SCORE
    // Score cone, pickup and score a cube
    public CommandBase scoreCone_1_Pickup_7_ScoreCube_11() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_1.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_1.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_1_2_3.getPath(drive),
                Path.READY_FOR_PICKUP_7.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_7.getPath(drive),
                Path.INTO_COMMUNITY_1_2_3.getPath(drive),
                Path.CUBE_SCORE_11.getPath(drive),
                scoreCube()

        )
                .withName("Score Cone (Pos 1) Pickup (Pos 7) Score Cube (Pos 11)");
    }

    public CommandBase scoreCone_2_Pickup_7_ScoreCube_11() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_2.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_2.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_1_2_3.getPath(drive),
                Path.READY_FOR_PICKUP_7.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_7.getPath(drive),
                Path.INTO_COMMUNITY_1_2_3.getPath(drive),
                Path.CUBE_SCORE_11.getPath(drive),
                scoreCube()

        )
                .withName("Score Cone (Pos 2) Pickup (Pos 7) Score Cube (Pos 11)");
    }

    public CommandBase scoreCone_3_Pickup_7_ScoreCube_11() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_3.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_3.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_1_2_3.getPath(drive),
                Path.READY_FOR_PICKUP_7.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_7.getPath(drive),
                Path.INTO_COMMUNITY_1_2_3.getPath(drive),
                Path.CUBE_SCORE_11.getPath(drive),
                scoreCube()

        )
                .withName("Score Cone (Pos 3) Pickup (Pos 7) Score Cube (Pos 11)");
    }

    public CommandBase scoreCone_1_Pickup_8_ScoreCube_11() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_1.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_1.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_1_2_3.getPath(drive),
                Path.READY_FOR_PICKUP_8.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_8.getPath(drive),
                Path.INTO_COMMUNITY_1_2_3.getPath(drive),
                Path.CUBE_SCORE_11.getPath(drive),
                scoreCube()

        )
                .withName("Score Cone (Pos 1) Pickup (Pos 8) Score Cube (Pos 11)");
    }

    public CommandBase scoreCone_2_Pickup_8_ScoreCube_11() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_2.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_2.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_1_2_3.getPath(drive),
                Path.READY_FOR_PICKUP_8.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_8.getPath(drive),
                Path.INTO_COMMUNITY_1_2_3.getPath(drive),
                Path.CUBE_SCORE_11.getPath(drive),
                scoreCube()

        )
                .withName("Score Cone (Pos 2) Pickup (Pos 8) Score Cube (Pos 11)");
    }

    public CommandBase scoreCone_3_Pickup_8_ScoreCube_11() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_3.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_3.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_1_2_3.getPath(drive),
                Path.READY_FOR_PICKUP_8.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_8.getPath(drive),
                Path.INTO_COMMUNITY_1_2_3.getPath(drive),
                Path.CUBE_SCORE_11.getPath(drive),
                scoreCube()

        )
                .withName("Score Cone (Pos 3) Pickup (Pos 8) Score Cube (Pos 11)");
    }

    public CommandBase scoreCone_1_Pickup_7_ScoreCube_12() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_1.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_1.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_1_2_3.getPath(drive),
                Path.READY_FOR_PICKUP_7.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_7.getPath(drive),
                Path.INTO_COMMUNITY_1_2_3.getPath(drive),
                Path.CUBE_SCORE_12.getPath(drive),
                scoreCube()

        )
                .withName("Score Cone (Pos 1) Pickup (Pos 7) Score Cube (Pos 12)");
    }

    public CommandBase scoreCone_2_Pickup_7_ScoreCube_12() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_2.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_2.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_1_2_3.getPath(drive),
                Path.READY_FOR_PICKUP_7.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_7.getPath(drive),
                Path.INTO_COMMUNITY_1_2_3.getPath(drive),
                Path.CUBE_SCORE_12.getPath(drive),
                scoreCube()

        )
                .withName("Score Cone (Pos 2) Pickup (Pos 7) Score Cube (Pos 12)");
    }

    public CommandBase scoreCone_3_Pickup_7_ScoreCube_12() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_3.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_3.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_1_2_3.getPath(drive),
                Path.READY_FOR_PICKUP_7.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_7.getPath(drive),
                Path.INTO_COMMUNITY_1_2_3.getPath(drive),
                Path.CUBE_SCORE_12.getPath(drive),
                scoreCube()

        )
                .withName("Score Cone (Pos 3) Pickup (Pos 7) Score Cube (Pos 12)");
    }

    public CommandBase scoreCone_1_Pickup_8_ScoreCube_12() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_1.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_1.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_1_2_3.getPath(drive),
                Path.READY_FOR_PICKUP_8.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_8.getPath(drive),
                Path.INTO_COMMUNITY_1_2_3.getPath(drive),
                Path.CUBE_SCORE_12.getPath(drive),
                scoreCube()

        )
                .withName("Score Cone (Pos 1) Pickup (Pos 8) Score Cube (Pos 12)");
    }

    public CommandBase scoreCone_2_Pickup_8_ScoreCube_12() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_2.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_2.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_1_2_3.getPath(drive),
                Path.READY_FOR_PICKUP_8.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_8.getPath(drive),
                Path.INTO_COMMUNITY_1_2_3.getPath(drive),
                Path.CUBE_SCORE_12.getPath(drive),
                scoreCube()

        )
                .withName("Score Cone (Pos 2) Pickup (Pos 8) Score Cube (Pos 12)");
    }

    public CommandBase scoreCone_3_Pickup_8_ScoreCube_12() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_3.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_3.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_1_2_3.getPath(drive),
                Path.READY_FOR_PICKUP_8.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_8.getPath(drive),
                Path.INTO_COMMUNITY_1_2_3.getPath(drive),
                Path.CUBE_SCORE_12.getPath(drive),
                scoreCube()

        )
                .withName("Score Cone (Pos 3) Pickup (Pos 8) Score Cube (Pos 12)");
    }

    public CommandBase scoreCone_4_Pickup_9_ScoreCube_12() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_4.getPath(drive),
                armExtend.moveTo(ArmPresets.HIGH_SCORE),
                scoreCone(),
                Path.BACKED_UP_4.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_4_5_6.getPath(drive),
                Path.READY_FOR_PICKUP_9.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_9.getPath(drive),
                Path.INTO_COMMUNITY_4_5_6.getPath(drive),
                Path.CUBE_SCORE_12.getPath(drive),
                scoreCube()

        )
                .withName("Score Cone (Pos 4) Pickup (Pos 9) Score Cube (Pos 12)");
    }

    public CommandBase scoreCone_5_Pickup_9_ScoreCube_12() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_5.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_5.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_4_5_6.getPath(drive),
                Path.READY_FOR_PICKUP_9.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_9.getPath(drive),
                Path.INTO_COMMUNITY_4_5_6.getPath(drive),
                Path.CUBE_SCORE_12.getPath(drive),
                scoreCube()

        )
                .withName("Score Cone (Pos 5) Pickup (Pos 9) Score Cube (Pos 12)");
    }

    public CommandBase scoreCone_6_Pickup_9_ScoreCube_12() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_6.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_6.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_4_5_6.getPath(drive),
                Path.READY_FOR_PICKUP_9.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_9.getPath(drive),
                Path.INTO_COMMUNITY_4_5_6.getPath(drive),
                Path.CUBE_SCORE_12.getPath(drive),
                scoreCube()

        )
                .withName("Score Cone (Pos 6) Pickup (Pos 9) Score Cube (Pos 12)");
    }

    public CommandBase scoreCone_4_Pickup_10_ScoreCube_12() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_4.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_4.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_4_5_6.getPath(drive),
                Path.READY_FOR_PICKUP_10.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_10.getPath(drive),
                Path.INTO_COMMUNITY_4_5_6.getPath(drive),
                Path.CUBE_SCORE_12.getPath(drive),
                scoreCube()

        )
                .withName("Score Cone (Pos 4) Pickup (Pos 10) Score Cube (Pos 12)");
    }

    public CommandBase scoreCone_5_Pickup_10_ScoreCube_12() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_5.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_5.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_4_5_6.getPath(drive),
                Path.READY_FOR_PICKUP_10.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_10.getPath(drive),
                Path.INTO_COMMUNITY_4_5_6.getPath(drive),
                Path.CUBE_SCORE_12.getPath(drive),
                scoreCube()

        )
                .withName("Score Cone (Pos 5) Pickup (Pos 10) Score Cube (Pos 12)");
    }

    public CommandBase scoreCone_6_Pickup_10_ScoreCube_12() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_6.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_6.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_4_5_6.getPath(drive),
                Path.READY_FOR_PICKUP_10.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_10.getPath(drive),
                Path.INTO_COMMUNITY_4_5_6.getPath(drive),
                Path.CUBE_SCORE_12.getPath(drive),
                scoreCube()

        )
                .withName("Score Cone (Pos 6) Pickup (Pos 10) Score Cube (Pos 12)");
    }

    public CommandBase scoreCone_4_Pickup_9_ScoreCube_13() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_4.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_4.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_4_5_6.getPath(drive),
                Path.READY_FOR_PICKUP_9.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_9.getPath(drive),
                Path.INTO_COMMUNITY_4_5_6.getPath(drive),
                Path.CUBE_SCORE_13.getPath(drive),
                scoreCube()

        )
                .withName("Score Cone (Pos 4) Pickup (Pos 9) Score Cube (Pos 13)");
    }

    public CommandBase scoreCone_5_Pickup_9_ScoreCube_13() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_5.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_5.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_4_5_6.getPath(drive),
                Path.READY_FOR_PICKUP_9.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_9.getPath(drive),
                Path.INTO_COMMUNITY_4_5_6.getPath(drive),
                Path.CUBE_SCORE_13.getPath(drive),
                scoreCube()

        )
                .withName("Score Cone (Pos 5) Pickup (Pos 9) Score Cube (Pos 13)");
    }

    public CommandBase scoreCone_6_Pickup_9_ScoreCube_13() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_6.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_6.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_4_5_6.getPath(drive),
                Path.READY_FOR_PICKUP_9.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_9.getPath(drive),
                Path.INTO_COMMUNITY_4_5_6.getPath(drive),
                Path.CUBE_SCORE_13.getPath(drive),
                scoreCube()

        )
                .withName("Score Cone (Pos 6) Pickup (Pos 9) Score Cube (Pos 13)");
    }

    public CommandBase scoreCone_4_Pickup_10_ScoreCube_13() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_4.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_4.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_4_5_6.getPath(drive),
                Path.READY_FOR_PICKUP_10.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_10.getPath(drive),
                Path.INTO_COMMUNITY_4_5_6.getPath(drive),
                Path.CUBE_SCORE_13.getPath(drive),
                scoreCube()

        )
                .withName("Score Cone (Pos 4) Pickup (Pos 10) Score Cube (Pos 13)");
    }

    public CommandBase scoreCone_5_Pickup_10_ScoreCube_13() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_5.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_5.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_4_5_6.getPath(drive),
                Path.READY_FOR_PICKUP_10.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_10.getPath(drive),
                Path.INTO_COMMUNITY_4_5_6.getPath(drive),
                Path.CUBE_SCORE_13.getPath(drive),
                scoreCube()

        )
                .withName("Score Cone (Pos 5) Pickup (Pos 10) Score Cube (Pos 13)");
    }

    public CommandBase scoreCone_6_Pickup_10_ScoreCube_13() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                prepareToScoreCone(),
                Path.UP_TO_CONE_STATION_6.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_6.getPath(drive),
                stowArmCloseIntake(),
                Path.OUT_OF_COMMUNITY_4_5_6.getPath(drive),
                Path.READY_FOR_PICKUP_10.getPath(drive),
                pickUpCube(),
                Path.GO_TO_PICKUP_10.getPath(drive),
                Path.INTO_COMMUNITY_4_5_6.getPath(drive),
                Path.CUBE_SCORE_13.getPath(drive),
                scoreCube()

        )
                .withName("Score Cone (Pos 6) Pickup (Pos 10) Score Cube (Pos 13)");
    }

    // Score cone and back up onto charge station (from pos 1) and then balance
    public CommandBase scoreConeBalance() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                armExtend.zeroVelocityCheck(),
                backUp(0.5, 0.5),
                armRotate.moveTo(ArmPresets.HIGH_SCORE),
                Path.UP_TO_CONE_STATION_1.getPath(drive),
                scoreCone(),
                Path.BACKED_UP_1.getPath(drive),
                stowArmCloseIntake(),
                Path.ON_CHARGE_STATION_14.getPath(drive)
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
                Path.PRE_TEST.getPath(drive)).withName("(TEST) Pre Test");
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
                Path.UP_TO_CONE_STATION_1.getPath(drive),
                armExtend.moveTo(ArmPresets.HIGH_SCORE),
                new FunctionalWaitCommand(() -> 0.25),
                intake.coneRelease(),
                new FunctionalWaitCommand(() -> 0.25),
                Path.BACKED_UP_1.getPath(drive),
                armExtend.moveTo(ArmPresets.ARM_STOWED),
                intake.coneGrab(),
                armRotate.moveTo(ArmPresets.ARM_STOWED),
                Path.OUT_OF_COMMUNITY_1_2_3.getPath(drive)

        )
                .withName("One Cone Taxi No Cable");
    }

    //// End of some more stuff that Joe wrote

}