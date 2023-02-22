package frc.robot.maps.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class FireLedMap {
    public AddressableLEDBuffer LED_BUFFER;
    public AddressableLED led;

    public FireLedMap() {
        this.led = new AddressableLED(0);
        this.LED_BUFFER = new AddressableLEDBuffer(0);

    }

    public FireLedMap(int ledPort, int ledBufferLength) {
        this.led = new AddressableLED(ledPort);
        this.LED_BUFFER = new AddressableLEDBuffer(ledBufferLength);
    }
}
