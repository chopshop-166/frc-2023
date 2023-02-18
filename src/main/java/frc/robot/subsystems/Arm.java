package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.subsystems.ArmMap;
import frc.robot.maps.subsystems.ArmMap.Data;

public class Arm extends SmartSubsystemBase {

    public Data data = new Data();
    public ArmMap extendMap;
    public final double SPEED = 0.3;
    private final double RETRACT_SPEED = -0.1;
    final double pivotHeight = 46.654;
    private double armAngle;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    DoublePublisher lengthPub = inst.getDoubleTopic("Arm/Length").publish();
    DoubleSubscriber angleSub = inst.getDoubleTopic("Arm/Angle").subscribe(0);

    public Arm(ArmMap extendMap) {
        this.extendMap = extendMap;
    }

    public boolean intakeBelowGround() {
        return pivotHeight - 5 < Math.cos(Math.toRadians(armAngle)) * (data.distanceInches + 42.3);

    }

    enum Level {
        // 0 in.
        LOW(0, 0),
        // 34 in.
        MEDIUM(0, 0),
        // 41 7/8 in.
        HIGH(0, 0);

        private double length;
        private double angle;

        private Level(double length, double angle) {
            this.length = length;
            this.angle = angle;
        }

        public double getLength() {
            return length;
        }

        public double getAngle() {
            return angle;
        }
    }

    // Manually sets the arm extension
    public CommandBase manual(DoubleSupplier motorSpeed) {
        return run(() -> {
            data.setPoint = limit(motorSpeed.getAsDouble() / 3);
        });
    }

    // Compares arm current extension to set distance and retracts or extends based
    // on current arm extenstion
    public CommandBase moveToDistanceBangBang(double distance, double speed) {
        return cmd("Move Distance").onExecute(() -> {
            if (distance >= data.distanceInches) {
                // Extend
                data.setPoint = limit(speed);

            } else {
                // Retract
                data.setPoint = limit(-speed);
            }
        }).runsUntil(() -> Math.abs(distance - data.distanceInches) < 0.5).onEnd(() -> {
            data.setPoint = 0;
        });
    }

    // Uses PID to change extension of arm to set distance
    public CommandBase moveToDistancePID(double distance) {
        return cmd("Move Distance").onExecute(() -> {
            // Extend
            data.setPoint = limit(extendMap.pid.calculate(data.distanceInches, distance));

        }).runsUntil(extendMap.pid::atSetpoint).onEnd(() -> {
            data.setPoint = 0;
        });
    }

    public CommandBase moveTo(Level level) {
        return moveToDistanceBangBang(level.getLength(), SPEED);
    }

    // This ensures that the arm is fully retracted (likely for the start or end of
    // a match)
    public CommandBase zeroVelocityCheck() {
        PersistenceCheck velocityPersistenceCheck = new PersistenceCheck(5,
                () -> Math.abs(data.velocityInchesPerSec) < 0.5);
        return cmd("Check Velocity").onInitialize(() -> {
            data.setPoint = limit(RETRACT_SPEED);
        }).runsUntil(velocityPersistenceCheck).onEnd(() -> {
            data.setPoint = 0;
            extendMap.extendMotor.getEncoder().reset();
        });
    }

    @Override
    public void reset() {
        // Nothing to reset here
        extendMap.extendMotor.getEncoder().reset();
    }

    @Override
    public void safeState() {
        // Sets arm extension point to 0
        data.setPoint = 0;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Use this for any background processing
        this.extendMap.updateData(data);
        Logger.getInstance().processInputs(getName(), data);
        lengthPub.set(data.distanceInches);
        armAngle = angleSub.get();
    }

    // Adds limits to arm extension speed

    private double limit(double speed) {
        if (speed > 0 && intakeBelowGround()) {
            return 0;
        }
        if ((data.distanceInches > extendMap.hardMaxDistance && speed > 0)
                || (data.distanceInches < extendMap.hardMinDistance && speed < 0)) {
            return data.setPoint = 0;
        }
        if ((data.distanceInches > extendMap.softMaxDistance && speed > 0)
                || (data.distanceInches < extendMap.softMinDistance && speed < 0)) {

            return speed * 0.2;
        }
        return speed;
    }
}