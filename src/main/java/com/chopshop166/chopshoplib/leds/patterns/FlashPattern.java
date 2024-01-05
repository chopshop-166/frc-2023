package com.chopshop166.chopshoplib.leds.patterns;

import com.chopshop166.chopshoplib.leds.AnimatedPattern;
import com.chopshop166.chopshoplib.leds.SegmentBuffer;

import edu.wpi.first.wpilibj.util.Color;

public class FlashPattern extends AnimatedPattern {
    private Color color;
    private boolean isOn;

    public FlashPattern(Color color, double delay) {
        // If super is called with a delay, then the pattern scheduler will call
        // update() after that many seconds have elapsed
        // Otherwise it runs update every frame
        super(delay);
        this.color = color;
    }

    @Override
    public void initialize(SegmentBuffer buffer) {
        isOn = true;
    }

    @Override
    public void animate(SegmentBuffer segment) {
        isOn = !isOn;
        segment.setAll(isOn ? this.color : Color.kBlack);
    }

    @Override
    public String toString() {
        return "FlashPattern(" + this.color.toString() + ")";
    }
}
