package frc.robot.util;

public class FireLeds {
    private static final int NUM_LEDS = 30;
    public final static byte[] heat = new byte[NUM_LEDS];

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

        // TODO: Add code to show LED colors
        // TODO: Add code to add delay
    }
}
