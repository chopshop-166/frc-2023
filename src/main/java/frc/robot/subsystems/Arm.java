package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.subsystems.ArmMap;
import frc.robot.maps.subsystems.ArmMap.Data;

public class Arm extends SmartSubsystemBase {

    public Data data = new Data();
    public ArmMap map;
    public final double SPEED = 0.3;
    private final double RETRACT_SPEED = -0.1;

    public Arm(ArmMap map) {
        this.map = map;
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
            data.setPoint = softLimit(motorSpeed.getAsDouble());
        });
    }

    // Compares arm current extension to set distance and retracts or extends based
    // on current arm extenstion
    public CommandBase moveToDistanceBangBang(double distance, double speed) {
        return cmd("Move Distance").onInitialize(() -> {
            if (distance >= data.distanceInches) {
                // Extend
                data.setPoint = softLimit(speed);
            } else {
                // Retract
                data.setPoint = softLimit(-speed);
            }
        }).runsUntil(() -> Math.abs(distance - data.distanceInches) < 0.5).onEnd(() -> {
            data.setPoint = 0;
        });
    }

    // Uses PID to change extension of arm to set distance
    public CommandBase moveToDistancePID(double distance) {
        return cmd("Move Distance").onExecute(() -> {
            // Extend
            data.setPoint = softLimit(map.pid.calculate(data.distanceInches, distance));

        }).runsUntil(map.pid::atSetpoint).onEnd(() -> {
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
            data.setPoint = softLimit(RETRACT_SPEED);
        }).runsUntil(velocityPersistenceCheck).onEnd(() -> {
            data.setPoint = 0;
            map.extendMotor.getEncoder().reset();
        });
    }

    @Override
    public void reset() {
        // Nothing to reset here
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
        this.map.updateData(data);
        Logger.getInstance().processInputs(getName(), data);
    }

    // Adds softlimit to arm extension speed
    private double softLimit(double speed) {
        if ((data.distanceInches > map.softMaxDistance && speed > 0) || (data.distanceInches < map.softMinDistance
                && speed < 0)) {
            return speed * 0.1;
        }
        return speed;
    }

}