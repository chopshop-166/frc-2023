package frc.robot.auto;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drive;

public enum AutoPath {

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
    // Pos. 14 -> inner side of chargestation (near scoring stations)
    // Pos. 15 -> outer side of chargestation (community)

    // CONE STATION SUB-POSITIONS
    UP_TO_CONE_STATION_1(0.05,
            new Pose2d(AutoConstants.UP_TO_CONE_X, 5.32, AutoConstants.ROTATION_180)),

    BACKED_UP_1(0.05,
            new Pose2d(AutoConstants.BACKED_UP_X, 5.32, AutoConstants.ROTATION_180)),

    UP_TO_CONE_STATION_2(0.05,
            new Pose2d(AutoConstants.UP_TO_CONE_X, 4.1, AutoConstants.ROTATION_180)),

    BACKED_UP_2(0.05,
            new Pose2d(AutoConstants.BACKED_UP_X, 4.15, AutoConstants.ROTATION_180)),

    UP_TO_CONE_STATION_3(0.05,
            new Pose2d(AutoConstants.UP_TO_CONE_X, 3.52, AutoConstants.ROTATION_180)),

    BACKED_UP_3(0.05,
            new Pose2d(AutoConstants.BACKED_UP_X, 3.55, AutoConstants.ROTATION_180)),

    UP_TO_CONE_STATION_4(0.05,
            new Pose2d(AutoConstants.UP_TO_CONE_X, 2.37, AutoConstants.ROTATION_180)),

    BACKED_UP_4(0.05,
            new Pose2d(AutoConstants.BACKED_UP_X, 2.39, AutoConstants.ROTATION_180)),

    UP_TO_CONE_STATION_5(0.05,
            new Pose2d(AutoConstants.UP_TO_CONE_X, 1.79, AutoConstants.ROTATION_180)),

    BACKED_UP_5(0.05,
            new Pose2d(AutoConstants.BACKED_UP_X, 1.81, AutoConstants.ROTATION_180)),

    UP_TO_CONE_STATION_6(0.05,
            new Pose2d(AutoConstants.UP_TO_CONE_X, 0.61, AutoConstants.ROTATION_180)),

    BACKED_UP_6(0.05,
            new Pose2d(AutoConstants.BACKED_UP_X, 0.63, AutoConstants.ROTATION_180)),

    //
    // CUBE STATION POSITIONS
    CUBE_SCORE_11(0.05,
            new Pose2d(AutoConstants.BACKED_UP_X, 4.71, AutoConstants.ROTATION_180)),

    CUBE_SCORE_12(0.05,
            new Pose2d(AutoConstants.BACKED_UP_X, 2.96, AutoConstants.ROTATION_180)),

    CUBE_SCORE_13(0.05,
            new Pose2d(AutoConstants.BACKED_UP_X, 1.21, AutoConstants.ROTATION_180)),

    // INTO AND OUT OF COMMUNITY
    OUT_OF_COMMUNITY_1_2_3(0.2,
            new Pose2d(2.0, 5.30, AutoConstants.ROTATION_180),
            new Pose2d(AutoConstants.OUT_OF_COMMUNITY_X, 5.30, AutoConstants.ROTATION_180)),

    OUT_OF_COMMUNITY_4_5_6(0.2,
            new Pose2d(2.0, 0.63, AutoConstants.ROTATION_180),
            new Pose2d(AutoConstants.OUT_OF_COMMUNITY_X, 0.63, AutoConstants.ROTATION_180)),

    INTO_COMMUNITY_1_2_3(0.2,
            new Pose2d(AutoConstants.OUT_OF_COMMUNITY_X, 5.30, AutoConstants.ROTATION_180),
            new Pose2d(2.0, 5.30, AutoConstants.ROTATION_180)),

    INTO_COMMUNITY_4_5_6(0.2,
            new Pose2d(AutoConstants.OUT_OF_COMMUNITY_X, 0.63, AutoConstants.ROTATION_180),
            new Pose2d(2.0, 0.63, AutoConstants.ROTATION_180)),

    //
    // PICKUP POSITIONS
    READY_FOR_PICKUP_7(0.2,
            new Pose2d(AutoConstants.OUT_OF_COMMUNITY_X, 5.30, AutoConstants.ROTATION_0),
            new Pose2d(AutoConstants.OUT_OF_COMMUNITY_X, 4.92, AutoConstants.ROTATION_0)),

    GO_TO_PICKUP_7(0.2,
            new Pose2d(6.90, 4.92, AutoConstants.ROTATION_0)),

    READY_FOR_PICKUP_8(0.2,
            new Pose2d(AutoConstants.OUT_OF_COMMUNITY_X, 5.30, AutoConstants.ROTATION_0),
            new Pose2d(AutoConstants.OUT_OF_COMMUNITY_X, 3.64, AutoConstants.ROTATION_0)),

    GO_TO_PICKUP_8(0.2,
            new Pose2d(6.90, 3.64, AutoConstants.ROTATION_0)),

    READY_FOR_PICKUP_9(0.2,
            new Pose2d(AutoConstants.OUT_OF_COMMUNITY_X, 0.63, AutoConstants.ROTATION_0),
            new Pose2d(AutoConstants.OUT_OF_COMMUNITY_X, 2.35, AutoConstants.ROTATION_0)),

    GO_TO_PICKUP_9(0.2,
            new Pose2d(6.90, 2.35, AutoConstants.ROTATION_0)),

    READY_FOR_PICKUP_10(0.2,
            new Pose2d(AutoConstants.OUT_OF_COMMUNITY_X, 0.63, AutoConstants.ROTATION_0),
            new Pose2d(AutoConstants.OUT_OF_COMMUNITY_X, 1.09, AutoConstants.ROTATION_0)),

    GO_TO_PICKUP_10(0.2,
            new Pose2d(6.90, 1.09, AutoConstants.ROTATION_0)),

    // CHARGE STATION POSTION(S?)
    INNER_SIDE_CHARGE_STATION_14(0.2,
            new Pose2d()),

    OUTER_SIDE_CHARGE_STATION_15(0.2,
            new Pose2d()),

    //// Start of some stuff that Joe wrote
    PRE_TEST(0.05,
            new Pose2d(14.165401626124469, 2.879876924969839, Rotation2d.fromRadians(-0.22656457143950046)));

    Pose2d poses[];
    double tolerance;

    private AutoPath(double tolerance, Pose2d... poses) {
        this.poses = poses;
        this.tolerance = tolerance;
    }

    // Create a sequence command to drive to each pose
    public CommandBase getPath(Drive drive) {
        return sequence(
                Arrays.stream(poses).map(pos -> drive.driveTo(pos, this.tolerance)).toArray(CommandBase[]::new))
                .withName(this.name()).andThen(drive.safeStateCmd());
    }
}
