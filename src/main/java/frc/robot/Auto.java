package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class Auto {
    // Declare references to subsystems
    Drive drive;

    // Pass in all subsystems
    public Auto(Drive drive) {
        this.drive = drive;
    }

    public CommandBase exampleAuto() {
        return sequence(
                drive.driveTo(new Pose2d(0.46558199999999994, 6.096132265165217, new Rotation2d(3.2194343382287403))),
                drive.driveTo(new Pose2d(2.7515797264691884, 4.148932802728144, new Rotation2d(4.6741400577246655))),
                drive.driveTo(new Pose2d(0.8889206444267342, 2.2438315546530774, new Rotation2d(6.27365135325216))),
                drive.driveTo(new Pose2d(0.888915333333334, 6.184039190724497, new Rotation2d(9.376577186399395))),
                drive.driveTo(new Pose2d(-1.9474170002695055, 6.773609234402517,
                        new Rotation2d(9.473503218655464))))
                .withName("Empty Auto");
    }

}