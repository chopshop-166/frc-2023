package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class Auto {
    // Declare references to subsystems
    Drive drive;

    // Pass in all subsystems
    public Auto(Drive drive) {
        this.drive = drive;
    }

    public enum Path {

        TEST(new Pose2d());

        Pose2d poses[];

        private Path(Pose2d... poses) {
            this.poses = poses;
        }

        // Create a sequence command to drive to each pose
        public CommandBase getPath(Drive drive) {
            return sequence(
                    Arrays.stream(poses).map(drive::driveTo).toArray(CommandBase[]::new)).withName(this.name());
        }
    }

    public CommandBase exampleAuto() {
        return sequence(Path.TEST.getPath(drive))
                .withName("Test Auto");
    }

}