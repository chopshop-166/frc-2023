package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.subsystems.ArmRotateMap;
import frc.robot.maps.subsystems.ArmRotateMap.Data;

public class ArmRotate extends SmartSubsystemBase {

    private ArmRotateMap map;
    final double MOVE_SPEED = 0.5;
    final double COMPARE_ANGLE = 5;
    final double SLOW_DOWN = 0.1;
    final double pivotHeight = 46.654;
    final PIDController pid;
    final Data data = new Data();
    private double armLength;

    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
    NetworkTable table = ntinst.getTable("ArmTable");

    public ArmRotate(ArmRotateMap map) {

        this.map = map;
        pid = map.pid;
    }

    public CommandBase move(DoubleSupplier rotationSpeed) {
        return run(() -> {
            data.setPoint = softLimit(rotationSpeed.getAsDouble() / 2);
        });
    }

    public boolean intakeBelowGround() {
        return pivotHeight > (Math.cos(data.degrees) * armLength);

    }

    public CommandBase moveToAngle(double angle) {
        // When executed the arm will move. The encoder will update until the desired
        // value is reached, then the command will end.
        return cmd("Move To Set Angle").onExecute(() -> {
            data.setPoint = pid.calculate(data.degrees, angle);

        }).runsUntil(pid::atSetpoint).onEnd(() -> {
            data.setPoint = 0;
        });

    }

    @Override
    public void reset() {
        map.motor.getEncoder().reset();
    }

    @Override
    public void safeState() {
        data.setPoint = 0;

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Use this for any background processing
        this.map.updateData(data);
        Logger.getInstance().processInputs(getName(), data);
        table.putValue("angle", NetworkTableValue.makeDouble(data.degrees));
        armLength = table.getValue("armLegnth").getDouble();
    }

    private double softLimit(double speed) {
        if (speed < 0 && intakeBelowGround()) {
            return 0;
        }
        if ((data.degrees > this.map.topAngle && speed > 0) ||
                (data.degrees < this.map.bottomAngle && speed < 0)) {
            return (speed * SLOW_DOWN);
        }
        return speed;
    }
}