package frc.robot.subsystems;

import com.chopshop166.chopshoplib.ColorMath;
import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.commands.FunctionalWaitCommand;
import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.IntakeData;

public class Intake extends LoggedSubsystem<IntakeData, IntakeData.Map> {

    private double armAngle;

    // Motor speed variables
    private final double GRAB_SPEED = 0.75;
    private final double RELEASE_SPEED = -0.45;

    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
    DoubleSubscriber angleSub = ntinst.getDoubleTopic("Arm/Angle").subscribe(0);
    BooleanPublisher clawPub = ntinst.getBooleanTopic("Intake/Closed").publish();

    // Creates constructor using IntakeData
    public Intake(IntakeData.Map map) {
        super(new IntakeData(), map);
    }

    @Override
    public void periodic() {
        super.periodic();
        armAngle = angleSub.get();
        clawPub.set(getData().solenoidSetPoint == Value.kForward);
    }

    // Releases game piece Cone
    public Command openIntake() {
        return runOnce(() -> {
            getData().solenoidSetPoint = Value.kReverse;
        });
    }

    public Command closeIntake() {
        return runOnce(() -> {
            getData().solenoidSetPoint = Value.kForward;
        });
    }

    // Opens or closes intake
    public Command toggle() {
        return runOnce(() -> {
            if (getData().solenoidSetPoint == Value.kForward) {
                if (armAngle > 13) {
                    getData().solenoidSetPoint = Value.kReverse;
                }
            } else {
                getData().solenoidSetPoint = Value.kForward;
            }
        });
    }

    public Command spinIn() {
        PersistenceCheck currentPersistenceCheck = new PersistenceCheck(5,
                () -> Math.abs(getData().currentAmps[0]) > 20);
        return cmd().onInitialize(
                () -> {
                    currentPersistenceCheck.reset();
                    getData().motorSetPoint = GRAB_SPEED;
                }).runsUntil(currentPersistenceCheck);
    }

    public void holdGamePiece(boolean interrupted) {
        getData().motorSetPoint = 0.05;
    }

    public Command grab() {
        return spinIn().andThen(new FunctionalWaitCommand(() -> 0.75).finallyDo(this::holdGamePiece));
    }

    // Releases game piece Cube
    public Command cubeRelease() {
        return runEnd(() -> {
            getData().motorSetPoint = RELEASE_SPEED;
        }, () -> {
            getData().motorSetPoint = 0;
        });
    }

    // Closes intake based on game piece detected
    public Command sensorControl() {

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