package com.chopshop166.chopshoplib.leds.patterns;

import com.chopshop166.chopshoplib.leds.Pattern;
import com.chopshop166.chopshoplib.leds.SegmentBuffer;

import edu.wpi.first.wpilibj.util.Color;

public class RainbowRoad extends Pattern {
    private int rainbowFirstPixelHue = 0;

    @Override
    public void initialize(SegmentBuffer buffer) {
        rainbowFirstPixelHue = 0;
    }

    @Override
    public void update(SegmentBuffer segment) {
        for (var i = 0; i < segment.getLength(); i++) {
            final int hue = (rainbowFirstPixelHue + (i * 180 / segment.getLength())) % 180;
            // Segment also allows setting individual lights by HSV, Color, or RGB
            segment.set(i, Color.fromHSV(hue, 255, 128));
        }
        rainbowFirstPixelHue += 177;
        rainbowFirstPixelHue %= 180;
    }

    @Override
    public String toString() {
        return "RainbowRoad()";
    }
}
