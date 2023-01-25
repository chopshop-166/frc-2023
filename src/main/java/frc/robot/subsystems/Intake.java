package frc.robot.subsystems;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import frc.robot.maps.subsystems.IntakeData;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Intake extends LoggedSubsystem<IntakeData, IntakeData.Map> {

    private final double GRAB_SPEED = 1;
    private final double RELEASE_SPEED = -1;

    public Intake(IntakeData.Map map) {
        super(new IntakeData(), map);
    }

    // grab command
    public CommandBase grab() {
        return runOnce(() -> {
            getData().setPoint = GRAB_SPEED;
        });
    }

    // release command
    public CommandBase release() {
        return runOnce(() -> {
            getData().setPoint = RELEASE_SPEED;
        });
    }

    @Override
    public void reset() {
        // Nothing to reset here
    }

    @Override
    public void safeState() {
        getData().setPoint = 0;
    }
}