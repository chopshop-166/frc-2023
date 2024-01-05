package com.chopshop166.chopshoplib.leds;

import edu.wpi.first.wpilibj.AddressableLED;

public class LedMap {
    public AddressableLED led;
    public LEDStripBuffer ledBuffer;

    public LedMap() {
        this.led = new AddressableLED(0);
        this.ledBuffer = new LEDStripBuffer(0);
    }

    public LedMap(int ledPort, int ledBufferLength) {
        this.led = new AddressableLED(ledPort);
        this.ledBuffer = new LEDStripBuffer(ledBufferLength);
    }
}
