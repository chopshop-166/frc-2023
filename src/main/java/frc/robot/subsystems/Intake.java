package frc.robot.subsystems;

import com.chopshop166.chopshoplib.ColorMath;
import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.subsystems.IntakeData;

public class Intake extends LoggedSubsystem<IntakeData, IntakeData.Map> {

    // Motor speed variables
    private final double GRAB_SPEED = 1;
    private final double RELEASE_SPEED = -1;

    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();

    BooleanPublisher clawPub = ntinst.getBooleanTopic("Intake/Closed").publish();

    // Creates constructor using IntakeData
    public Intake(IntakeData.Map map) {
        super(new IntakeData(), map);
    }

    @Override
    public void periodic() {
        super.periodic();
        clawPub.set(getData().solenoidSetPoint == Value.kForward);
    }

    // Grabs game piece Cone
    public CommandBase coneGrab() {
        return runOnce(() -> {
            getData().solenoidSetPoint = Value.kForward;
        });
    }

    // Releases game piece Cone
    public CommandBase coneRelease() {
        return runOnce(() -> {
            getData().solenoidSetPoint = Value.kReverse;
        });
    }

    // Grabs game piece Cube
    public CommandBase cubeGrab() {
        return runEnd(() -> {
            getData().motorSetPoint = GRAB_SPEED;
        }, () -> {
            getData().motorSetPoint = 0;
        });
    }

    // Releases game piece Cube
    public CommandBase cubeRelease() {
        return runEnd(() -> {
            getData().motorSetPoint = RELEASE_SPEED;
        }, () -> {
            getData().motorSetPoint = 0;
        });
    }

    // Closes intake based on game piece detected
    public CommandBase sensorControl() {

        return cmd().onExecute(() -> {
            if (getData().gamePieceDistance <= getData().maxGamePieceDistance &&
                    getData().gamePieceDistance >= getData().minGamePieceDistance) {
                if (ColorMath.equals(getData().sensorColor, Color.kPurple, .2)) {
                    getData().motorSetPoint = GRAB_SPEED;
                } else if (ColorMath.equals(getData().sensorColor, Color.kYellow, .2)) {
                    getData().solenoidSetPoint = Value.kForward;
                }
            }
        }).runsUntil(() -> getData().gamePieceDistance <= getData().minGamePieceDistance).onEnd(() -> {
            safeState();
        });

    }

    @Override
    public void reset() {
        // Nothing to reset here
    }

    @Override
    public void safeState() {
        // Turns off the motors
        getData().motorSetPoint = 0;

        // Stops movement of air
        getData().solenoidSetPoint = Value.kOff;
    }
}