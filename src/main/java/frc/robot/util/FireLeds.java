package frc.robot.util;

public class FireLeds {
    public class FireEffect {
        private static final int NUM_LEDS = 30;
        private static byte[] heat = new byte[NUM_LEDS];

        public static void fire(int flameHeight, int sparks, int delayDuration) {
            int cooldown;

            // Cool down each cell a little
            for (int i = 0; i < NUM_LEDS; i++) {
                cooldown = (int) (Math.random() * ((flameHeight * 10) / NUM_LEDS + 2));

                if (cooldown > heat[i]) {
                    heat[i] = 0;
                } else {
                    heat[i] = (byte) (heat[i] - cooldown);
                }
            }

            // Heat from each cell drifts up and diffuses slightly
            for (int k = NUM_LEDS - 1; k >= 2; k--) {
                heat[k] = (byte) ((heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3);
            }

            // Randomly ignite new sparks near bottom of the flame
            if (Math.random() * 255 < sparks) {
                int y = (int) (Math.random() * 7);
                heat[y] = (byte) (heat[y] + (int) (Math.random() * (160 - 255 + 1) + 160));
            }

            // Convert heat to LED colors
            for (int j = 0; j < NUM_LEDS; j++) {
                setPixelHeatColor(j, heat[j]);
            }

            // TODO: Add code to show LED colors
            // TODO: Add code to add delay
        }

        private static void setPixelHeatColor(int pixel, byte temperature) {
            // Rescale heat from 0-255 to 0-191
            byte t192 = (byte) Math.round((temperature / 255.0) * 191);

            // Calculate ramp up from
            byte heatramp = (byte) (t192 & 0x3F); // 0...63
            heatramp <<= 2; // scale up to 0...252

            // Figure out which third of the spectrum we're in:
            if (t192 > (byte) 0x80) { // hottest
                // TODO: Add code to set RGB color of the LED
            } else if (t192 > (byte) 0x40) { // middle
                // TODO: Add code to set RGB color of the LED
            } else { // coolest
                // TODO: Add code to set RGB color of the LED
            }
        }
    }

}
