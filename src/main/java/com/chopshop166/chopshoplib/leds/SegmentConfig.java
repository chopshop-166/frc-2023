package com.chopshop166.chopshoplib.leds;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class SegmentConfig {
    /* package */ int length;
    /* package */ Set<String> tagSet = new HashSet<>();
    /* package */ boolean isReversed;
    /* package */ int startPos;
    /* package */ int stopPos;
    // Note that the originals store references to their mirrors to make it easier
    // to update
    /* package */ List<SegmentConfig> mirrors = new ArrayList<>();
    private LEDStripBuffer buffer;

    /* package */ SegmentConfig(LEDStripBuffer buffer, int startPos, int length) {
        this.buffer = buffer;
        this.startPos = startPos;
        this.stopPos = startPos + length - 1;
        this.length = length;
    }

    public SegmentConfig reversed(boolean isReversed) {
        this.isReversed = isReversed;
        return this;
    }

    public SegmentConfig tags(String... newTags) {
        tagSet.addAll(Arrays.asList(newTags));
        buffer.addTags(this, newTags);
        return this;
    }

    public void update(int i, Color c, AddressableLEDBuffer buf) {
        buf.setLED(indexFor(i), c);
        for (var mirror : mirrors) {
            mirror.update(i, c, buf);
        }
    }

    private int indexFor(int i) {
        if (isReversed) {
            return stopPos - i;
        } else {
            return startPos + i;
        }
    }

    @Override
    public String toString() {
        return "SegmentConfig(" + startPos + "," + stopPos + ")";
    }

}
