package frc.robot.maps;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.chopshop166.chopshoplib.drive.SwerveDriveMap;
import com.chopshop166.chopshoplib.maps.RobotMapFor;
// $Imports$
import frc.robot.maps.subsystems.IntakeData;
import frc.robot.maps.subsystems.DriveMap;

import frc.robot.maps.subsystems.SwerveDriveMap;

import frc.robot.maps.subsystems.LedMap;

@RobotMapFor("Default")
public class RobotMap {
    // $Maps$

    SwerveDriveMap drive = new SwerveDriveMap();

    // $Getters$
    public IntakeData.Map getIntakeMap() {
        return new IntakeData.Map();
    }
    public DriveMap getDriveMap() {
        return driveMap;
    public SwerveDriveMap getDriveMap() {
        return new SwerveDriveMap();
    }

    public SwerveDriveMap getDriveMap() {
        return drive;
    }

    public LedMap getLedMap() {
        return new LedMap();
    }

    public void SetupLogging() {
        // Pull the replay log from AdvantageScope (or prompt the user)
        String logPath = LogFileUtil.findReplayLog();
        // Read replay log
        Logger.getInstance().setReplaySource(new WPILOGReader(logPath));
        // Save outputs to a new log
        Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    }
}
