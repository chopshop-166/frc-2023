package com.chopshop166.chopshoplib.leds;

import java.util.List;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class SegmentBuffer {

    private AddressableLEDBuffer buf;
    private List<SegmentConfig> confs;

    /* package */ SegmentBuffer(AddressableLEDBuffer buf, List<SegmentConfig> confs) {
        this.buf = buf;
        this.confs = confs;
    }

    public void set(int index, Color c) {
        int realIndex = index;
        for (SegmentConfig conf : confs) {
            if (realIndex < conf.length) {
                conf.update(realIndex, c, buf);
                return;
            } else {
                realIndex -= conf.length;
            }
        }
    }

    public void setAll(Color c) {
        for (SegmentConfig conf : confs) {
            for (int i = 0; i < conf.length; i++) {
                conf.update(i, c, buf);
            }
        }
    }

    public int getLength() {
        return confs.stream().mapToInt(c -> c.length).sum();
    }

    @Override
    public String toString() {
        return "SegmentBuffer(" + confs.stream().toList().toString() + ")";
    }
}
