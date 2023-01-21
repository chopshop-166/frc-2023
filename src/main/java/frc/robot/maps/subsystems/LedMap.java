package frc.robot.maps.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LedMap {
    public AddressableLEDBuffer ledBuffer;
    public AddressableLED led;

    public LedMap() {
        this.led = new AddressableLED(0);
        this.ledBuffer = new AddressableLEDBuffer(0);
    }

    public LedMap(AddressableLED led, AddressableLEDBuffer ledBuffer) {
        this.led = led;
        this.ledBuffer = ledBuffer;
    }
}
