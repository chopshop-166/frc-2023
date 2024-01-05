package com.chopshop166.chopshoplib.leds.patterns;

import com.chopshop166.chopshoplib.leds.Pattern;
import com.chopshop166.chopshoplib.leds.SegmentBuffer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class SpinPattern extends Pattern {
    private Timer timer = new Timer();
    private int ledPosition;

    @Override
    public void initialize(SegmentBuffer buffer) {
        ledPosition = 0;
        timer.start();
    }

    @Override
    public void update(SegmentBuffer buffer) {
        if (timer.advanceIfElapsed(0.05)) {
            ledPosition++;
        }
        if (ledPosition == buffer.getLength()) {
            ledPosition = 0;
        }
        buffer.setAll(Color.kBlack);
        buffer.set(ledPosition, Color.kGreen);
    }

    @Override
    public String toString() {
        return "SpinPattern()";
    }
}
