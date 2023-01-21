package frc.robot;

import org.littletonrobotics.junction.Logger;

import java.util.PrimitiveIterator.OfInt;

import javax.annotation.meta.When;

import com.chopshop166.chopshoplib.Autonomous;
import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap;
// $Imports$
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Led;

public class Robot extends CommandRobot {

    private RobotMap map = getMapForName("OffAxis", RobotMap.class, "frc.robot.maps");
    private ButtonXboxController driveController = new ButtonXboxController(0);
    private RobotMap map = new RobotMap();
    private Led led = new Led(map.getLedMap());

    private ButtonXboxController controller = new ButtonXboxController(0);
    // $Subsystems$
    private Drive drive = new Drive(map.getDriveMap());

    private Auto auto = new Auto();

    @Autonomous(defaultAuto = true)
    public CommandBase exampleAuto = auto.exampleAuto();

    @Override
    public void teleopInit() {

    }

    @Override
    public void robotInit() {
        super.robotInit();

        Logger.getInstance().recordMetadata("ProjectName", "FRC-2023"); // Set a metadata value
        map.SetupLogging();
        if (!isReal()) {
            setUseTiming(false); // Run as fast as possible
        }
        // Start logging! No more data receivers, replay sources, or metadata values may
        // be added.
        Logger.getInstance().start();
    }

    @Override
    public void configureButtonBindings() {
        controller.a().onTrue(led.setYellow());
        controller.b().onTrue(led.setPurple());
        controller.x().onTrue(led.resetColor());
        controller.y().onTrue(led.printColor());
    }

    @Override
    public void populateDashboard() {

    }

    @Override
    public void setDefaultCommands() {
        drive.setDefaultCommand(
                drive.drive(driveController::getLeftX, driveController::getLeftY, driveController::getRightX));
    }

        led.setDefaultCommand(led.colorAlliance());
    }
}