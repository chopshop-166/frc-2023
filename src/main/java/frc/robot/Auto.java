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

        BEFORE_CONE_STATION(0.1,
                new Pose2d(14.56, 1.48, Rotation2d.fromDegrees(0))),

        UP_TO_CONE_STATION(0.05,
                new Pose2d(14.83, 1.42, Rotation2d.fromDegrees(0))),
        OUT_OF_COMMUNITY(0.1,
                new Pose2d(14.04, 3.86, Rotation2d.fromDegrees(0)),
                new Pose2d(10.58, 3.92, Rotation2d.fromDegrees(0))

        ),

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

    public CommandBase exampleAuto() {
        return sequence(Path.TEST.getPath(drive))
                .withName("Test Auto");
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