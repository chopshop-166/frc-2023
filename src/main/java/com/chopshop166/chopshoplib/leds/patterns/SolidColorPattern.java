package com.chopshop166.chopshoplib.leds.patterns;

import com.chopshop166.chopshoplib.leds.Pattern;
import com.chopshop166.chopshoplib.leds.SegmentBuffer;

import edu.wpi.first.wpilibj.util.Color;

public class SolidColorPattern extends Pattern {
    private Color color;

    public SolidColorPattern(Color color) {
        this.color = color;
    }

    @Override
    public void initialize(SegmentBuffer segment) {
        // update is given an SegmentBuffer object
        // This object maps numbers 0..N to the appropriate numbers in all segments with
        // a given tag, in creation order
        // For convenience there's a setAll to set every LED in the segment
        segment.setAll(this.color);
    }

    @Override
    public String toString() {
        return "SolidColorPattern(" + this.color.toString() + ")";
    }
}
