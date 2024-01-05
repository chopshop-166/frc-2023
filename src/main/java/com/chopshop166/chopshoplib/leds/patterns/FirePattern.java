package com.chopshop166.chopshoplib.leds.patterns;

import com.chopshop166.chopshoplib.leds.Pattern;
import com.chopshop166.chopshoplib.leds.SegmentBuffer;

import edu.wpi.first.wpilibj.util.Color;

public class FirePattern extends Pattern {
    private final byte[] heat;

    public FirePattern(int length) {
        heat = new byte[length];
    }

    @Override
    public void initialize(SegmentBuffer buffer) {
        // Do nothing
    }

    @Override
    public void update(SegmentBuffer buffer) {
        calculateFire(30);
        for (int i = 1; i < heat.length; i++) {
            buffer.set(i, heatToColor(heat[i]));
        }
    }

    protected Color heatToColor(byte h) {
        // Scale 'heat' down from 0-255 to 0-191
        byte t192 = (byte) Math.round((h / 255.0) * 191);

        // calculate ramp up from
        byte heatramp = (byte) (t192 & 0x3F); // 0..63
        heatramp <<= 2; // scale up to 0..252

        // figure out which third of the spectrum we're in:
        if (t192 > 0x80) { // hottest
            return new Color(255, 255, heatramp);
        } else if (t192 > 0x40) { // middle
            return new Color(255, heatramp, 0);
        } else { // coolest
            return new Color(heatramp, 0, 0);
        }
    }

    private void calculateFire(int sparks) {
        // Cool down each cell a little
        for (int i = 1; i < heat.length; i++) {
            int cooldown = (int) (Math.random() * ((heat.length * 10) / heat.length + 2));

            if (cooldown > heat[i]) {
                heat[i] = 0;
            } else {
                heat[i] = (byte) (heat[i] - cooldown);
            }
        }

        // Heat from each cell drifts up and diffuses slightly
        for (int k = heat.length - 1; k >= 2; k--) {
            heat[k] = (byte) ((heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3);
        }

        // Randomly ignite new sparks near bottom of the flame
        if (Math.random() * 255 < sparks) {
            int y = (int) (Math.random() * 7);
            heat[y] = (byte) (heat[y] + (int) (Math.random() * (160 - 255 + 1) + 160));
        }
    }

    @Override
    public String toString() {
        return "FirePattern()";
    }
}
