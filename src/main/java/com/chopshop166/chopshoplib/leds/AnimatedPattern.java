package com.chopshop166.chopshoplib.leds;

import edu.wpi.first.wpilibj.Timer;

public abstract class AnimatedPattern extends Pattern {

    private final Timer timer = new Timer();
    private final double delay;

    public AnimatedPattern(double delay) {
        this.delay = delay;
    }

    public abstract void animate(SegmentBuffer buffer);

    @Override
    public void update(SegmentBuffer buffer) {
        if (timer.advanceIfElapsed(delay)) {
            animate(buffer);
        }
    }
}
