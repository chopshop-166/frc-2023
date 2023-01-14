package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {

    RobotPoseEstimator estimator;

    public Vision(String cameraName, AprilTagFieldLayout aprilTags, Transform3d cameraToRobot) {
        PhotonCamera camera = new PhotonCamera(NetworkTableInstance.getDefault(), cameraName);
        List<Pair<PhotonCamera, Transform3d>> cameras = new ArrayList<>();
        cameras.add(new Pair<>(camera, cameraToRobot));
        estimator = new RobotPoseEstimator(
                aprilTags, PoseStrategy.AVERAGE_BEST_TARGETS, cameras);
    }

    public Pose2d update() {
        return estimator.update().get().getFirst().toPose2d();
    }
}
