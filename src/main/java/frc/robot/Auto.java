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

        UP_TO_CONE_STATION(0.05,
                new Pose2d(1.7539,
                        5.0123, Rotation2d.fromDegrees(180))),

        BACKED_UP(0.05,
                new Pose2d(2.1,
                        5.0123, Rotation2d.fromDegrees(180))),

        OUT_OF_COMMUNITY(0.2,
                new Pose2d(2.1877, 4.7327, Rotation2d.fromDegrees(180)),
                new Pose2d(5.1150, 4.7327, Rotation2d.fromDegrees(0))

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

    public CommandBase oneConeAuto() {
        return sequence(
                // armRotate.zeroVelocityCheck(),
                armExtend.zeroVelocityCheck(),

                armRotate.moveTo(EnumLevel.HIGH_SCORE),
                Path.UP_TO_CONE_STATION.getPath(drive),
                armExtend.moveTo(EnumLevel.HIGH_SCORE),
                new FunctionalWaitCommand(() -> 0.25),
                intake.coneRelease(),
                new FunctionalWaitCommand(() -> 0.25),
                Path.BACKED_UP.getPath(drive),
                armExtend.moveTo(EnumLevel.ARM_STOWED),
                intake.coneGrab(),
                armRotate.moveTo(EnumLevel.ARM_STOWED),
                Path.OUT_OF_COMMUNITY.getPath(drive)

        )
                .withName("One Cone Auto");
    }

}