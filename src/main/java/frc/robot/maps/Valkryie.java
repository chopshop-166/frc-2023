package frc.robot.maps;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.maps.subsystems.LedMap;

public class Valkryie extends RobotMap {
    @Override
    public LedMap getLedMap() {
        AddressableLED led = new AddressableLED(0);
        AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(30);

        return new LedMap(led, ledBuffer);
    }

}
