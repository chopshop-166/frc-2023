package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Field {

    // Wrapper for loading an AprilTagFieldLayout from a JSON file
    public static AprilTagFieldLayout getApriltagLayout() {
        String absolutePath = Filesystem.getDeployDirectory().getAbsolutePath();

        AprilTagFieldLayout layout = new AprilTagFieldLayout(new ArrayList<>(), 0, 0);
        Path jsonPath = Path.of(absolutePath + "/apriltags.json");
        try {
            layout = new AprilTagFieldLayout(jsonPath);
            String outp = "";
            for (AprilTag tag : layout.getTags()) {
                outp += tag.ID + ", ";
            }
            SmartDashboard.putString("List Of Tags", outp);
        } catch (IOException exception) {
            System.err.println("April Tag Layout JSON Not Found");
            SmartDashboard.putString("List Of Tags", exception.toString());
        }

        return layout;
    }
}
