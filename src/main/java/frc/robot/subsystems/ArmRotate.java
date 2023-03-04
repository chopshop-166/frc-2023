package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.EnumLevel;
import frc.robot.maps.subsystems.ArmRotateMap;
import frc.robot.maps.subsystems.ArmRotateMap.Data;

public class ArmRotate extends SmartSubsystemBase {

    private ArmRotateMap map;
    final double MOVE_SPEED = 0.5;
    final double RAISE_SPEED = 0.5;
    final double LOWER_SPEED = 0.4;
    final double COMPARE_ANGLE = 5;
    final double SLOW_DOWN = 0.2;
    final double PIVOT_HEIGHT = 46.654;
    private final double INTAKE_DEPTH_LIMIT = 0;
    private final double DESCEND_SPEED = -0.3;
    final double armStartLength = 42.3;
    final double NO_FALL = 0.02;
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
            double speedCoef = RAISE_SPEED;
            if (rotationSpeed.getAsDouble() < 0) {
                speedCoef = LOWER_SPEED;
            }
            data.setPoint = limits(rotationSpeed.getAsDouble() * speedCoef);
        });
    }

    public boolean intakeBelowGround() {
        double armZ = (Math.cos(Math.toRadians(data.degrees)) * (armLength + armStartLength));

        return PIVOT_HEIGHT - INTAKE_DEPTH_LIMIT < armZ;

    }

    public CommandBase moveToAngle(double angle) {
        // When executed the arm will move. The encoder will update until the desired
        // value is reached, then the command will end.
        PersistenceCheck setPointPersistenceCheck = new PersistenceCheck(20, pid::atSetpoint);
        return cmd("Move To Set Angle").onExecute(() -> {
            data.setPoint = pid.calculate(data.degrees, angle) + NO_FALL;
            Logger.getInstance().recordOutput("getPositionErrors", pid.getPositionError());

        }).runsUntil(setPointPersistenceCheck).onEnd(() -> {
            data.setPoint = NO_FALL;
        });

    }

    public CommandBase zeroVelocityCheck() {
        PersistenceCheck velocityPersistenceCheck = new PersistenceCheck(1,
                () -> data.acceleration > 2);
        return cmd("Check Velocity").onInitialize(() -> {
            velocityPersistenceCheck.reset();
            data.setPoint = DESCEND_SPEED;
        }).runsUntil(velocityPersistenceCheck).onEnd(() -> {
            data.setPoint = 0;
            map.motor.getEncoder().reset();
        });
    }

    public CommandBase resetZero(DoubleSupplier speed) {
        return cmd().onExecute(() -> {
            data.setPoint = DESCEND_SPEED;
        }).onEnd(() -> {
            map.motor.getEncoder().reset();
        });
    }

    public CommandBase moveTo(EnumLevel level) {
        return moveToAngle(level.getAngle());
    }

    public CommandBase resetAngle() {
        return cmd().onInitialize(() -> {
            reset();
        }).runsUntil(() -> {
            return true;
        }).runsWhenDisabled(true);
    }

    public CommandBase brakeMode() {
        return cmd().onInitialize(() -> {
            map.setBrake();
        }).runsUntil(() -> {
            return true;
        }).runsWhenDisabled(true);
    }

    public CommandBase coastMode() {
        return cmd().onInitialize(() -> {
            map.setCoast();
        }).runsUntil(() -> {
            return true;
        }).runsWhenDisabled(true);
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
        Logger.getInstance().recordOutput("speed", speed);
        if (speed < 0 && intakeBelowGround()) {
            return NO_FALL;
        }

        if (!intakeSub.get() && speed < 0 && data.degrees < map.bumperAngle) {
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
        if (data.degrees > 13) {
            return (speed + NO_FALL);
        }

        return speed;
    }
}