package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.time.Instant;
import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.maps.subsystems.ArmMap;

public class Arm extends SmartSubsystemBase {

    private ArmMap map;
    private SmartMotorController motor = new SmartMotorController();
    PIDController pid = new PIDController(0, 0, 0);

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

    public final double SPEED = 1;

    public CommandBase extend(DoubleSupplier motorSpeed) {
        return run(() -> {
            motor.set(motorSpeed.getAsDouble());
        });
    }

    public CommandBase moveToDistance(double distance, double speed) {
        return cmd("Move Distance").onInitialize(() -> {
            if (distance >= motor.getEncoder().getDistance()) {
                // extend
                motor.set(speed);
            } else {
                // retract
                motor.set(-speed);
            }
        }).runsUntil(() -> Math.abs(distance - motor.getEncoder().getDistance()) < 0.5).onEnd(() -> {
            motor.stopMotor();
        });
    }

    public CommandBase movetoLow() {
        return moveToDistance(Level.LOW.get(), 2);
    }

    public CommandBase movetoMedium() {
        return moveToDistance(Level.MEDIUM.get(), 2);
    }

    public CommandBase movetoHigh() {
        return moveToDistance(Level.HIGH.get(), 2);
    }

    public CommandBase rotate() {
        return runOnce(() -> {

        });
    }

    @Override
    public void reset() {
        // Nothing to reset here
    }

    @Override
    public void safeState() {

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Use this for any background processing
    }

    // Will I need this? I don't think so but I'm not deleting it
    public CommandBase retract(DoubleSupplier motorSpeed) {
        return run(() -> {
            motor.set(motorSpeed.getAsDouble());
        });
    }

    public CommandBase retractDistance(double distance, double speed) {
        return cmd("Retract Distance").onInitialize(() -> {
            motor.set(speed);
        }).runsUntil(() -> {
            if (distance == motor.getEncoder().getDistance()) {
                motor.stopMotor();
                return true;
            } else {
                return false;
            }
        });
    }
}