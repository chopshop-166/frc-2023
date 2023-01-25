package frc.robot.subsystems;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import frc.robot.maps.subsystems.IntakeMap;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Intake extends SmartSubsystemBase {

    private IntakeMap map;

    public Intake(IntakeMap map) {
        this.map = map;
    }
 
    //on command
    public CommandBase open() 

            
     

     
    //off command

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