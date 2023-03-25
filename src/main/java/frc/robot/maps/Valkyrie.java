package frc.robot.maps;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.motors.CSSparkMax;
import com.chopshop166.chopshoplib.sensors.MockEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.maps.subsystems.ArmExtendMap;
import frc.robot.maps.subsystems.ArmRotateMap;
import frc.robot.maps.subsystems.LedMap;

@RobotMapFor("Valkyrie")
public class Valkyrie extends RobotMap {

    @Override
    public LedMap getLedMap() {
        return new LedMap(0, 30);
    }

    @Override
    public ArmRotateMap getArmRotateMap() {
        CSSparkMax motor = new CSSparkMax(18, MotorType.kBrushless);
        PIDController pid = new PIDController(0.01, 0, 0);
        DutyCycleEncoder mockArmEncoder = new DutyCycleEncoder(18);
        pid.setTolerance(1);
        return new ArmRotateMap(motor, 1, 10, 1, 10, 0, pid, new MockEncoder(), 0, 0);
    }

    @Override
    public ArmExtendMap getArmMap() {
        CSSparkMax motor = new CSSparkMax(9, MotorType.kBrushless);
        motor.getMotorController().setIdleMode(IdleMode.kBrake);
        PIDController pidController = new PIDController(0, 0, 0);
        pidController.setTolerance(4);
        return new ArmExtendMap(motor, 400, 20, 400, 20, pidController, 46.654, 42.3);
    }

    @Override
    public void setupLogging() {
        Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
        Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        Logger.getInstance().recordMetadata("RobotMap", this.getClass().getSimpleName());
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    }
}
