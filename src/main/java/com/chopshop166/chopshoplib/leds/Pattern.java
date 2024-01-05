package com.chopshop166.chopshoplib.leds;

public abstract class Pattern {

    public Pattern() {
        //
    }

    public abstract void initialize(SegmentBuffer buffer);

    public void update(SegmentBuffer buffer) {
    }
}
