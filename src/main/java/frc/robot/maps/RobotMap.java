package frc.robot.maps;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.chopshop166.chopshoplib.maps.RobotMapFor;

// $Imports$
import frc.robot.maps.subsystems.ArmMap;
import frc.robot.maps.subsystems.ArmRotateMap;
import frc.robot.maps.subsystems.IntakeData;
import frc.robot.maps.subsystems.LedMap;
import frc.robot.maps.subsystems.SwerveDriveMap;

@RobotMapFor("Default")
public class RobotMap {

    // $Getters$
    public ArmMap getArmMap() {
        return new ArmMap();
    }

    public ArmRotateMap getArmRotateMap() {
        return new ArmRotateMap();
    }

    public IntakeData.Map getIntakeMap() {
        return new IntakeData.Map();
    }

    public SwerveDriveMap getDriveMap() {
        return new SwerveDriveMap();
    }

    public LedMap getLedMap() {
        return new LedMap();
    }

    public void setupLogging() {
        // Pull the replay log from AdvantageScope (or prompt the user)
        String logPath = LogFileUtil.findReplayLog();
        // Read replay log
        Logger.getInstance().setReplaySource(new WPILOGReader(logPath));
        // Save outputs to a new log
        Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    }
}
