package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.Arrays;

import com.chopshop166.chopshoplib.commands.FunctionalWaitCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmRotate;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;

public class Auto {
    // Declare references to subsystems
    Drive drive;
    Arm armExtend;
    ArmRotate armRotate;
    Intake intake;

    // Pass in all subsystems
    public Auto(Drive drive, Arm armExtend, ArmRotate armRotate, Intake intake) {
        this.drive = drive;
        this.armExtend = armExtend;
        this.armRotate = armRotate;
        this.intake = intake;
    }

    public enum Path {

        // 1.913324824060518,1.8401481550095398,202.15111118447658
        // 1.536021824789258,1.8071871015354626,162.72159242120003

        BEFORE_CONE_STATION(0.1,
                new Pose2d(1.913, 1.840, Rotation2d.fromDegrees(202.151))),

        UP_TO_CONE_STATION(0.05,
                new Pose2d(1.536, 1.807, Rotation2d.fromDegrees(162.722))),

        OUT_OF_COMMUNITY(0.2,
                new Pose2d(1.787, 0.621, Rotation2d.fromDegrees(172.156)),
                new Pose2d(5.903, 0.621, Rotation2d.fromDegrees(173.884))

        ),
        BALANCE(0.1,
                new Pose2d(2.008, 2.354, Rotation2d.fromDegrees(182.885)),
                new Pose2d(3.836, 2.000, Rotation2d.fromDegrees(
                        203.851))),

        TEST(0.1,
                new Pose2d(14.38, 3.45, Rotation2d.fromDegrees(90)),
                new Pose2d(12.38, 3.45, Rotation2d.fromDegrees(180))

        );

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

    public CommandBase testConeLOne() {
        return sequence(drive.setPose(new Pose2d(4.72, 15.37, new Rotation2d(180))), intake.coneToggle(),
                drive.driveTo(new Pose2d(4.72, 10.43, new Rotation2d(0))), armRotate.moveToAngle(20), intake.cubeGrab(),
                drive.driveTo(new Pose2d(5.32, 15.37, new Rotation2d(180))), armRotate.moveTo(EnumLevel.HIGH_SCORE))
                .withName("TEST-ConeL1");
    }

    public CommandBase oneConeAuto() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                armExtend.zeroVelocityCheck(),

                Path.BEFORE_CONE_STATION.getPath(drive),
                armRotate.moveTo(EnumLevel.HIGH_SCORE),
                Path.UP_TO_CONE_STATION.getPath(drive),
                armExtend.moveTo(EnumLevel.HIGH_SCORE),
                new FunctionalWaitCommand(() -> 0.25),
                intake.coneRelease(),
                new FunctionalWaitCommand(() -> 0.25),
                Path.BEFORE_CONE_STATION.getPath(drive),
                armExtend.moveTo(EnumLevel.ARM_STOWED),
                intake.coneGrab(),
                armRotate.moveTo(EnumLevel.ARM_STOWED),
                Path.OUT_OF_COMMUNITY.getPath(drive)

        )
                .withName("Test Auto");
    }

}