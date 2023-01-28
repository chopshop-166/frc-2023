package frc.robot.subsystems;

import com.chopshop166.chopshoplib.ColorMath;
import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import frc.robot.maps.subsystems.IntakeData;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Intake extends LoggedSubsystem<IntakeData, IntakeData.Map> {

    private final double GRAB_SPEED = 1;
    private final double RELEASE_SPEED = -1;

    public Intake(IntakeData.Map map) {
        super(new IntakeData(), map);
    }

    public CommandBase ConeGrab() {
        return runOnce(() -> {
            getData().solenoidSetPoint = Value.kForward;
        });
    }

    public CommandBase ConeRelease() {
        return runOnce(() -> {
            getData().solenoidSetPoint = Value.kReverse;
        });
    }

    public CommandBase CubeGrab() {
        return runOnce(() -> {
            getData().motorSetPoint = GRAB_SPEED;
        });
    }

    public CommandBase CubeRelease() {
        return runOnce(() -> {
            getData().motorSetPoint = RELEASE_SPEED;
        });
    }

    public CommandBase sensorControl() {
        return run(() -> {

            if (ColorMath.equals(getData().sensorColor, Color.kBlue, .2)
                    && getData().gamePieceDistance <= getData().maxGamePieceDistance
                    && getData().gamePieceDistance >= getData().minGamePieceDistance) {
                cmd().onInitialize(() -> {
                    getData().motorSetPoint = GRAB_SPEED;
                }).runsUntil(() -> getData().gamePieceDistance <= getData().minGamePieceDistance).onEnd(() -> {
                    safeState();
                });
            }

            if (ColorMath.equals(getData().sensorColor, Color.kYellow, .2)
                    && getData().gamePieceDistance <= getData().maxGamePieceDistance
                    && getData().gamePieceDistance >= getData().minGamePieceDistance) {
                cmd().onInitialize(() -> {
                    getData().solenoidSetPoint = Value.kForward;
                }).runsUntil(() -> getData().gamePieceDistance <= getData().minGamePieceDistance).onEnd(() -> {
                    safeState();
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
        getData().motorSetPoint = 0;
        getData().solenoidSetPoint = Value.kOff;
    }
}