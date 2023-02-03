package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.fasterxml.jackson.annotation.JacksonInject.Value;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.maps.subsystems.ArmMap;
import frc.robot.maps.subsystems.ArmMap.Data;

public class Arm extends SmartSubsystemBase {

    public Data data = new Data();
    public ArmMap map;

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

        public double get() {
            return length;
        }

        public double getAngle() {
            return angle;
        }
    }

    public final double SPEED = 0.3;

    public CommandBase manual(DoubleSupplier motorSpeed) {
        return run(() -> {
            data.setPoint = softLimit(motorSpeed.getAsDouble());
        });
    }

    public CommandBase moveToDistanceBangBang(double distance, double speed) {
        return cmd("Move Distance").onInitialize(() -> {
            if (distance >= data.distanceInches) {
                // extend
                data.setPoint = softLimit(speed);
            } else {
                // retract
                data.setPoint = softLimit(-speed);
            }
        }).runsUntil(() -> Math.abs(distance - data.distanceInches) < 0.5).onEnd(() -> {
            data.setPoint = 0;
        });
    }

    public CommandBase moveToDistancePID(double distance) {
        return cmd("Move Distance").onExecute(() -> {
            // extend
            data.setPoint = softLimit(map.pid.calculate(data.distanceInches, distance));

        }).runsUntil(() -> Math.abs(distance - data.distanceInches) < 4).onEnd(() -> {
            data.setPoint = 0;
        });
    }

    public CommandBase moveTo(Level level) {
        return moveToDistanceBangBang(level.get(), SPEED);
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
        if ((data.distanceInches > map.softMaxDistance && speed > 0) || (data.distanceInches < map.softMinDistance
                && speed < 0)) {
            return speed * 0.1;
        }
        return speed;
    }

}