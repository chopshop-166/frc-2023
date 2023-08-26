package frc.robot;

/**
 * Presets for arm extension length and angle
 *
 * length is stored in inches
 * angle is stored in degrees
 */
public enum ArmPresets {
    // Need to measure values
    MEDIUM_SCORE(3.8, 78,
            78),

    HIGH_SCORE(18.5, 95,
            87),

    MEDIUM_SCORE_DOWN(3.8, 62,
            58),

    HIGH_SCORE_DOWN(18.5, 80,
            76.3),

    // Human Player Station = HPS
    HPS_PICKUP(0, 77,
            72),

    ARM_STOWED(1, 2, 1),

    CUBE_PICKUP(4, 18, 18),

    CONE_PICKUP(8.9, 33, 33);

    private double length;
    private double angle;
    private double absoluteAngle;

    private ArmPresets(double length, double angle, double absoluteAngle) {
        this.length = length;
        this.angle = angle;
        this.absoluteAngle = absoluteAngle;
    }

    /**
     * Get the arm extension length
     *
     * @return Arm extension length in inches
     */
    public double getLength() {
        return length;
    }

    /**
     * Get the angle of the arm
     *
     * @return Arm angle in degrees
     */
    public double getAngle() {
        return getAngle(false);
    }

    public double getAngle(boolean absolute) {
        return absolute ? absoluteAngle : angle;
    }

}
