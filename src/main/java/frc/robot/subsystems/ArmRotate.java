package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.subsystems.ArmRotateMap;
import frc.robot.maps.subsystems.ArmRotateMap.Data;

public class ArmRotate extends SmartSubsystemBase {

    private ArmRotateMap map;
    final double MOVE_SPEED = 0.5;
    final double COMPARE_ANGLE = 5;
    final double SLOW_DOWN = 0.1;
    final PIDController pid;
    final Data data = new Data();

    public ArmRotate(ArmRotateMap map) {

        this.map = map;
        pid = map.pid;
    }

    public CommandBase move(DoubleSupplier rotationSpeed) {
        return run(() -> {
            data.setPoint = softLimit(rotationSpeed.getAsDouble());
        });
    }

    public CommandBase moveToAngle(double angle) {
        // When executed the arm will move. The encoder will update until the desired
        // value is reached, then the command will end.
        return cmd("Move To Set Angle").onExecute(() -> {
            data.setPoint = pid.calculate(data.degrees, angle);

        }).runsUntil(() -> (pid.atSetpoint())).onEnd(() -> {
            data.setPoint = 0;
        });

    }

    @Override
    public void reset() {
        // Nothing to reset here
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
    }

    private double softLimit(double speed) {
        if ((data.degrees > this.map.topAngle && speed > 0) ||
                (data.degrees < this.map.bottomAngle && speed < 0)) {
            return (speed * SLOW_DOWN);
        }
        return speed;
    }
}