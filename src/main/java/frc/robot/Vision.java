package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.maps.subsystems.SwerveDriveMap;

public class Vision {

    RobotPoseEstimator estimator;
    SwerveDriveMap driveMap;
    SwerveDriveOdometry odometry;
    PhotonCamera camera;
    Transform3d cameraToRobot;
    AprilTagFieldLayout aprilTags;

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                new SwerveModulePosition(0, driveMap.frontLeft().getAngle()),
                new SwerveModulePosition(0, driveMap.frontRight().getAngle()),
                new SwerveModulePosition(0, driveMap.rearLeft().getAngle()),
                new SwerveModulePosition(0, driveMap.rearRight().getAngle()),
        };
    }

    public Vision(String cameraName, AprilTagFieldLayout aprilTags, Transform3d cameraToRobot,
            SwerveDriveMap driveMap) {
        camera = new PhotonCamera(NetworkTableInstance.getDefault(), cameraName);
        this.driveMap = driveMap;
        this.cameraToRobot = cameraToRobot;
        this.aprilTags = aprilTags;

        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                driveMap.frontLeft().getLocation(),
                driveMap.frontRight().getLocation(),
                driveMap.rearLeft().getLocation(),
                driveMap.rearRight().getLocation());

        odometry = new SwerveDriveOdometry(kinematics, driveMap.gyro().getRotation2d(),
                getModulePositions());

    }

    public Pose2d update() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d cameraToTarget = target.getBestCameraToTarget();
            int tagId = target.getFiducialId();
            Pose2d pose = aprilTags.getTagPose(tagId).get().plus(cameraToTarget.inverse())
                    .plus(cameraToRobot.inverse()).toPose2d();

            driveMap.gyro().setAngle(pose.getRotation().getDegrees());
            odometry.resetPosition(driveMap.gyro().getRotation2d(),
                    getModulePositions(), pose);
        }

        return odometry.update(driveMap.gyro().getRotation2d(), getModulePositions());
    }
}
