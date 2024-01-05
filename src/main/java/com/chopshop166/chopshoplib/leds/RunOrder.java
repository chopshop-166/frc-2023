package com.chopshop166.chopshoplib.leds;

/* package */ record RunOrder(Pattern pattern, SegmentBuffer buffer) {

    public void initialize() {
        pattern.initialize(buffer);
    }

    public void update() {
        pattern.update(buffer);
    }
}
