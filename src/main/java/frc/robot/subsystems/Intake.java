package frc.robot.subsystems;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;
import com.chopshop166.chopshoplib.pneumatics.IDSolenoid;
import com.chopshop166.chopshoplib.pneumatics.MockDSolenoid;
import com.chopshop166.chopshoplib.sensors.IColorSensor;

import frc.robot.maps.subsystems.IntakeData;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Intake extends LoggedSubsystem<IntakeData, IntakeData.Map> {

    private IDSolenoid solenoid = new MockDSolenoid();
    private final double GRAB_SPEED = 1;
    private final double RELEASE_SPEED = -1;
    private IColorSensor colorSensor;

    public Intake(IntakeData.Map map) {
        super(new IntakeData(), map);
    }

    public CommandBase ConeGrab() {
        return runOnce(() -> {
            solenoid.set(Value.kForward);
        });
    }

    public CommandBase ConeRelease() {
        return runOnce(() -> {
            solenoid.set(Value.kReverse);
        });
    }

    public CommandBase CubeGrab() {
        return runOnce(() -> {
            getData().setPoint = GRAB_SPEED;
        });
    }

    public CommandBase CubeRelease() {
        return runOnce(() -> {
            getData().setPoint = RELEASE_SPEED;
        });
    }

    public CommandBase sensorControl() {
        return run(() -> {
            Color DetectedColor = colorSensor.getColor();

            if (DetectedColor == Color.kBlue) {
                cmd().onInitialize(() -> {
                    getData().setPoint = GRAB_SPEED;
                }).runsUntil(() -> DetectedColor != Color.kBlue).onEnd(() -> {
                    getData().setPoint = RELEASE_SPEED;
                });
            }

            if (DetectedColor == Color.kYellow) {
                cmd().onInitialize(() -> {
                    solenoid.set(Value.kForward);
                }).runsUntil(() -> DetectedColor != Color.kYellow).onEnd(() -> {
                    solenoid.set(Value.kReverse);
                });
            }

        });

    }

    @Override
    public void reset() {
        // Nothing to reset here
    }

    @Override
    public void safeState() {
        getData().setPoint = 0;
        solenoid.set(Value.kOff);
    }
}