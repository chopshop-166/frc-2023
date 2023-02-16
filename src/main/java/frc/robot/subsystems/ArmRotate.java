package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.subsystems.ArmRotateMap;
import frc.robot.maps.subsystems.ArmRotateMap.Data;

public class ArmRotate extends SmartSubsystemBase {

    private ArmRotateMap map;
    final double MOVE_SPEED = 0.5;
    final double COMPARE_ANGLE = 5;
    final double SLOW_DOWN = 0.2;
    final double pivotHeight = 46.654;
    final double armStartLength = 42.3;
    final double noFall = 0.03;
    final PIDController pid;
    final Data data = new Data();
    private double armLength;

    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();

    DoubleSubscriber lengthSub = ntinst.getDoubleTopic("Arm/Length").subscribe(0);
    DoublePublisher anglePub = ntinst.getDoubleTopic("Arm/Angle").publish();
    BooleanSubscriber intakeSub = ntinst.getBooleanTopic("Intake/Closed").subscribe(false);

    public ArmRotate(ArmRotateMap map) {

        this.map = map;
        pid = map.pid;
    }

    public CommandBase move(DoubleSupplier rotationSpeed) {
        return run(() -> {
            data.setPoint = limits(rotationSpeed.getAsDouble() + noFall);
        });
    }

    public boolean intakeBelowGround() {
        double armZ = (Math.cos(Math.toRadians(data.degrees)) * (armLength + armStartLength));

        return pivotHeight < armZ;

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
        anglePub.set(data.degrees);
        armLength = lengthSub.get();
    }

    private double limits(double speed) {
        if (speed < 0 && intakeBelowGround()) {
            return 0;
        }

        if (intakeSub.get() && speed < 0 && data.degrees < map.bumperAngle) {
            return 0;
        }
        if ((data.degrees > this.map.hardMaxAngle && speed > 0)
                || (data.degrees < this.map.hardMinAngle && speed < 0)) {
            return data.setPoint = 0;
        }
        if ((data.degrees > this.map.softMaxAngle && speed > 0) ||
                (data.degrees < this.map.softMinAngle && speed < 0)) {
            return (speed * SLOW_DOWN);
        }

        return speed;
    }
}