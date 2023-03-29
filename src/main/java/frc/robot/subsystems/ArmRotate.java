package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ArmPresets;
import frc.robot.maps.subsystems.ArmRotateMap;
import frc.robot.maps.subsystems.ArmRotateMap.Data;

public class ArmRotate extends SmartSubsystemBase {

    private ArmRotateMap map;
    final double RAISE_SPEED = 0.5;
    final double LOWER_SPEED = 0.4;
    final double SLOW_DOWN = 0.2;
    final double PIVOT_HEIGHT = 46.654;
    private final double INTAKE_DEPTH_LIMIT = 0;
    private final double DESCEND_SPEED = -0.3;
    final double armStartLength = 42.3;
    final double NO_FALL = 0.024;
    final ProfiledPIDController pid;
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
        double armZ = (Math.cos(Math.toRadians(data.rotatingRelativeAngleDegrees)) * (armLength + armStartLength));

        return PIVOT_HEIGHT - INTAKE_DEPTH_LIMIT < armZ;

    }

    public CommandBase moveToAngle(double angle) {
        // When executed the arm will move. The encoder will update until the desired
        // value is reached, then the command will end.
        PersistenceCheck setPointPersistenceCheck = new PersistenceCheck(20, pid::atGoal);
        return cmd("Move To Set Angle").onInitialize(() -> {
            pid.reset(data.rotatingRelativeAngleDegrees, data.rotatingAngleVelocity);
        }).onExecute(() -> {
            data.setPoint = pid.calculate(data.rotatingRelativeAngleDegrees, angle) + NO_FALL;
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

    public CommandBase resetZero() {
        return runEnd(() -> {
            data.setPoint = DESCEND_SPEED;
        }, () -> {
            map.motor.getEncoder().reset();
            data.setPoint = 0;
        });
    }

    public CommandBase moveTo(ArmPresets level) {
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
        return new InstantCommand(() -> {
            map.setBrake();
        }).ignoringDisable(true);
    }

    public CommandBase coastMode() {
        return new InstantCommand(() -> {
            map.setCoast();
        }).ignoringDisable(true);
    }

    @Override
    public void reset() {
        map.motor.getEncoder().reset();
        map.encoder.reset();
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
        anglePub.set(data.rotatingRelativeAngleDegrees);
        armLength = lengthSub.get();
    }

    private double limits(double speed) {
        Logger.getInstance().recordOutput("speed", speed);
        if (speed < 0 && intakeBelowGround()) {
            return NO_FALL;
        }

        if (!intakeSub.get() && speed < 0 && data.rotatingRelativeAngleDegrees < map.bumperAngle) {
            return 0;
        }
        if ((data.rotatingRelativeAngleDegrees > this.map.hardMaxAngle && speed > 0)
                || (data.rotatingRelativeAngleDegrees < this.map.hardMinAngle && speed < 0)) {
            return data.setPoint = 0;
        }
        if ((data.rotatingRelativeAngleDegrees > this.map.softMaxAngle && speed > 0) ||
                (data.rotatingRelativeAngleDegrees < this.map.softMinAngle && speed < 0)) {
            return (speed * SLOW_DOWN);
        }
        if (data.rotatingRelativeAngleDegrees > 13) {
            return (speed + NO_FALL);
        }

        return speed;
    }
}