package frc.robot;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.maps.subsystems.SwerveDriveMap;
import frc.robot.util.PoseFilter;

public class Vision {

    SwerveDriveMap driveMap;
    SwerveDriveOdometry odometry;
    PhotonCamera camera;
    Transform3d cameraToRobot;
    AprilTagFieldLayout aprilTags;
    PoseFilter filter = new PoseFilter(0.1);
    Pose2d prevPose = new Pose2d();
    public boolean sawTag = false;

    public Vision(
            String cameraName, AprilTagFieldLayout aprilTags,
            Transform3d cameraToRobot,
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

    public void setPose(Pose2d pose) {
        odometry.resetPosition(Rotation2d.fromDegrees(driveMap.gyro().getAngle() - 180), getModulePositions(), pose);
    }

    // Estimated pose from a combination of vision and odometry
    public Pose2d update() {
        PhotonPipelineResult result = camera.getLatestResult();

        // Sees an apriltag

        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d cameraToTarget = target.getBestCameraToTarget();
            int tagId = target.getFiducialId();
            SmartDashboard.putNumber("Tag ID", tagId);
            Optional<Pose3d> opt = aprilTags.getTagPose(tagId);

            if (opt.isPresent() && target.getPoseAmbiguity() < 0.3) {
                // Reverse the pose to determine the position on the field
                Pose2d pose = opt.get().plus(cameraToTarget.inverse())
                        .plus(cameraToRobot.inverse()).toPose2d();

                double tagDistance = cameraToTarget.getTranslation().getDistance(new Translation3d());

                double distance = 0;
                if (sawTag) {
                    distance = prevPose.getTranslation().getDistance(pose.getTranslation());
                } else {
                    driveMap.gyro().setAngle(pose.getRotation().getDegrees());
                }
                if (distance < 2 && tagDistance < 4) {
                    setPose(pose);
                }
                sawTag = true;
            }
        }

        SmartDashboard.putBoolean("Saw Tag", sawTag);
        // Subtract 180 degrees from the gyro angle for some reason

        prevPose = filter.calculate(odometry.update(
                Rotation2d.fromDegrees(driveMap.gyro().getAngle()), getModulePositions()));
        return prevPose;
    }

    // Get every swerve module state
    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                new SwerveModulePosition(driveMap.frontLeft().getDistance(), driveMap.frontLeft().getAngle()),
                new SwerveModulePosition(driveMap.frontRight().getDistance(), driveMap.frontRight().getAngle()),
                new SwerveModulePosition(driveMap.rearLeft().getDistance(), driveMap.rearLeft().getAngle()),
                new SwerveModulePosition(driveMap.rearRight().getDistance(), driveMap.rearRight().getAngle()),
        };
    }
}
