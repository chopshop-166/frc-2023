package com.chopshop166.chopshoplib.leds;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDStripBuffer {
    private List<SegmentConfig> segmentConfigs = new ArrayList<>();
    private Map<String, Set<SegmentConfig>> segmentTagMap = new HashMap<>();
    private Map<SegmentConfig, Pattern> patternMap = new HashMap<>();
    private List<RunOrder> runOrder = new ArrayList<>();
    private Set<Pattern> newPatterns = new HashSet<>();
    private int numLEDs;
    private int nextLED;
    private final AddressableLEDBuffer buffer;

    public LEDStripBuffer(int numLEDs) {
        this.numLEDs = numLEDs;
        this.nextLED = 0;
        buffer = new AddressableLEDBuffer(numLEDs);
    }

    public int getLength() {
        return numLEDs;
    }

    public SegmentConfig segment(int length) {
        assert numLEDs >= nextLED + length;
        assert length > 0;
        var result = new SegmentConfig(this, nextLED, length);
        segmentConfigs.add(result);
        nextLED += length;
        return result;
    }

    public SegmentConfig mirrorSegment(SegmentConfig origConfig) {
        var result = segment(origConfig.length);
        origConfig.mirrors.add(result);
        return result;
    }

    public void addTags(SegmentConfig config, String... tags) {
        for (String tag : tags) {
            segmentTagMap.putIfAbsent(tag, new HashSet<>());
            segmentTagMap.get(tag).add(config);
        }
    }

    public void setGlobalPattern(Pattern pattern) {
        for (var config : segmentConfigs) {
            patternMap.put(config, pattern);
        }
        // Then it triggers a recalculation of run order
        recalculateRunOrder();
        newPatterns.add(pattern);
    }

    public void setPattern(String tag, Pattern pattern) {
        // When setPattern is called, it removes the existing pattern from any segments
        // that match those tags
        // and then uses the given pattern in their place
        //
        // Internally, the scheduler has a Map<String, List<SegmentConfig>>
        // When setPattern is called, it indexes into that map and replaces each data's
        // active pattern with this new one
        for (var config : segmentTagMap.getOrDefault(tag, new HashSet<>())) {
            patternMap.put(config, pattern);
        }
        // Then it triggers a recalculation of run order
        recalculateRunOrder();
        newPatterns.add(pattern);
    }

    public void update(AddressableLED led) {
        for (var order : runOrder) {
            if (newPatterns.contains(order.pattern())) {
                order.initialize();
            }
        }
        newPatterns.clear();
        // When leds.update() is called, it goes into the run order and calls the
        // pattern's update with the segment buffer
        for (var order : runOrder) {
            order.update();
        }
        led.setData(buffer);
    }

    private void recalculateRunOrder() {
        // Run order is a List<RunOrder>
        // When calculated, it groups together every segment with the same pattern, in
        // creation order
        Set<Pattern> matchedPatterns = new HashSet<>();
        runOrder.clear();
        for (int i = 0; i < segmentConfigs.size(); i++) {
            Pattern p = patternMap.get(segmentConfigs.get(i));
            if (p != null) {
                // If we haven't checked this pattern already
                if (!matchedPatterns.contains(p)) {
                    // Group together all segments with the same pattern
                    List<SegmentConfig> patternConfigs = new ArrayList<>();
                    for (int j = i; j < segmentConfigs.size(); j++) {
                        var conf = segmentConfigs.get(j);
                        Pattern p2 = patternMap.get(conf);
                        if (p == p2) {
                            patternConfigs.add(conf);
                        }
                    }
                    // Create a Segment Buffer from the list
                    var segbuf = new SegmentBuffer(buffer, patternConfigs);
                    runOrder.add(new RunOrder(p, segbuf));
                }
                matchedPatterns.add(p);
            }
        }
    }
}
