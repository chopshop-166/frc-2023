package frc.robot.maps.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LedMap {
    public AddressableLEDBuffer ledBuffer;
    public AddressableLED led;

    public LedMap() {
        this.led = new AddressableLED(0);
        this.ledBuffer = new AddressableLEDBuffer(30);
    }

    public LedMap(int ledPort, int ledBufferLength) {
        this.led = new AddressableLED(ledPort);
        this.ledBuffer = new AddressableLEDBuffer(ledBufferLength);
    }
}
