package com.chopshop166.chopshoplib.leds.patterns;

import edu.wpi.first.wpilibj.util.Color;

public class ColdFirePattern extends FirePattern {

    public ColdFirePattern(int length) {
        super(length);
    }

    @Override
    protected Color heatToColor(byte h) {
        // Scale 'heat' down from 0-255 to 0-191
        byte t192 = (byte) Math.round((h / 255.0) * 191);

        // calculate ramp up from
        byte heatramp = (byte) (t192 & 0x3F); // 0..63
        heatramp <<= 2; // scale up to 0..252

        // figure out which third of the spectrum we're in:
        if (t192 > 0x80) { // hottest
            return new Color(heatramp, 255, 255);
        } else if (t192 > 0x40) { // middle
            return new Color(0, heatramp, 255);
        } else { // coolest
            return new Color(0, 0, heatramp);
        }
    }

    @Override
    public String toString() {
        return "ColdFirePattern()";
    }
}
