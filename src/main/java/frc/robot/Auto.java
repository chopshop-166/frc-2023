package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Auto {
    // Declare references to subsystems

    // Pass in all subsystems
    Auto() {
        // Assign all subsystems to local storage
    }

    public CommandBase exampleAuto() {
        return runOnce(() -> {
        }).withName("Empty Auto");
    }

}