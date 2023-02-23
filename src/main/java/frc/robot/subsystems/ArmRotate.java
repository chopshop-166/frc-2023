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
    private final double INTAKE_DEPTH_LIMIT = 5;
    private final double DESCEND_SPEED = -0.1;
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

    enum Level {
        // 0 in.
        LOW(0),
        // 34 in.
        MEDIUM(0),
        // 41 7/8 in.
        HIGH(0);

        private double angle;

        private Level(double angle) {
            this.angle = angle;
        }

        public double getAngle() {
            return angle;
        }
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

    public CommandBase zeroVelocityCheck() {
        PersistenceCheck velocityPersistenceCheck = new PersistenceCheck(5,
                () -> Math.abs(data.velocityDegreesPerSecond) < 0.5);
        return cmd("Check Velocity").onInitialize(() -> {
            data.setPoint = DESCEND_SPEED;
        }).runsUntil(velocityPersistenceCheck).onEnd(() -> {
            data.setPoint = 0;
            map.motor.getEncoder().reset();
        });
    }

    public CommandBase moveToAngleBangBang(double angle, double speed) {
        return cmd("Move To Set Angle").onExecute(() -> {
            if (angle >= data.degrees) {
                data.setPoint = limits(speed);
            } else {
                data.setPoint = limits(-speed);
            }
        }).runsUntil(() -> Math.abs(angle - data.degrees) < 2.0).onEnd(() -> {
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