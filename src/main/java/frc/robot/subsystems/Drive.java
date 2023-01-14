package frc.robot.subsystems;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Field;
import frc.robot.Vision;
import frc.robot.maps.subsystems.DriveMap;

public class Drive extends SmartSubsystemBase {

    private DriveMap map;

    private Vision vision;
    private Pose2d pose = new Pose2d();

    public Drive(DriveMap map) {
        this.map = map;
        vision = new Vision(
                "gloworm", Field.getApriltagLayout(),
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(0),
                                Units.inchesToMeters(0),
                                Units.inchesToMeters(0)),
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0))));
    }

    @Override
    public void reset() {
        // Nothing to reset here
    }

    @Override
    public void safeState() {

    }

    @Override
    public void periodic() {
        pose = vision.update();
    }
}