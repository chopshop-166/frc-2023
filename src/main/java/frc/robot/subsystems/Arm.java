package frc.robot.subsystems;

import java.time.Instant;
import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.subsystems.ArmMap;

public class Arm extends SmartSubsystemBase {

    private ArmMap map;
    private SmartMotorController motor = new SmartMotorController();

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

        public double getAgain() {
            return angle;
        }
    }

    public CommandBase rotate() {
        return instant("Rotate Arm", () -> {

        });
    }

    // This stuff is important (assigned this part) \/
    //
    public final double SPEED = 1;;

    public CommandBase extend(DoubleSupplier motorSpeed) {
        return instant("Extend Arm", () -> {
            motor.set(SPEED);
        });
    }

    public CommandBase extendDistance(double distance, double speed) {
        return cmd("Extend Distance").onInitialize(() -> {
            motor.set(distance);
        });
    }

    public CommandBase retract(DoubleSupplier motorSpeed) {
        return instant("Extend Arm", () -> {
            motor.set(SPEED);
        });
    }

    public CommandBase retractDistance(double distance, double speed) {
        return cmd("Extend Distance").onInitialize(() -> {
            motor.set(distance);
        });
    }

    // End of important stuff
    //
    public CommandBase extendLow() {

    }

    public CommandBase retract() {
        return instant("Retract Arm", () -> {

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
}