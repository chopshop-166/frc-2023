package frc.robot.maps;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.motors.CSSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.maps.subsystems.ArmMap;
import frc.robot.maps.subsystems.LedMap;

@RobotMapFor("Valkyrie")
public class Valkyrie extends RobotMap {

    @Override
    public ArmMap getArmMap() {
        CSSparkMax motor = new CSSparkMax(9, MotorType.kBrushless);
        motor.getMotorController().setIdleMode(IdleMode.kBrake);
        return new ArmMap(motor, 400, 20, new PIDController(0.005, 0, 0));
    }

    @Override
    public LedMap getLedMap() {
        return new LedMap(0, 30);
    }

    @Override
    public void SetupLogging() {
        Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
        Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        Logger.getInstance().recordMetadata("RobotMap", this.getClass().getSimpleName());
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    }
}
