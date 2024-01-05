package com.chopshop166.chopshoplib.leds.patterns;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.leds.Pattern;
import com.chopshop166.chopshoplib.leds.SegmentBuffer;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Color;

public class IndicatorPattern extends Pattern {
    private final Color trueColor;
    private final Color falseColor;
    private final BooleanSupplier source;

    public IndicatorPattern(final Color trueColor, final Color falseColor, final BooleanSupplier supplier) {
        this.trueColor = trueColor;
        this.falseColor = falseColor;
        this.source = supplier;
    }

    public IndicatorPattern(final Color trueColor, final Color falseColor, final String topic) {
        final NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
        final BooleanSubscriber table = ntinst.getBooleanTopic(topic).subscribe(false);
        this.trueColor = trueColor;
        this.falseColor = falseColor;
        this.source = table::get;
    }

    @Override
    public void initialize(SegmentBuffer buffer) {
        // Do nothing
    }

    @Override
    public void update(SegmentBuffer buffer) {
        buffer.setAll(source.getAsBoolean() ? trueColor : falseColor);
    }

    @Override
    public String toString() {
        return "IndicatorPattern(" + this.trueColor.toString() + ", " + this.falseColor.toString() + ")";
    }
}
