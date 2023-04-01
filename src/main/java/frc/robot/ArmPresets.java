package frc.robot;

/**
 * Presets for arm extension length and angle
 *
 * length is stored in inches
 * angle is stored in degrees
 */
public enum ArmPresets {
    // Need to measure values
    MEDIUM_SCORE(3.8, 78),

    HIGH_SCORE(18.5, 86),

    MEDIUM_SCORE_ACTUAL(3.8, 62),

    HIGH_SCORE_ACTUAL(18.5, 76.3),

    // Human Player Station = HPS
    HPS_PICKUP(0, 72),

    ARM_STOWED(1, 2),

    CUBE_PICKUP(3, 17),

    CONE_PICKUP(8.9, 33);

    private double length;
    private double angle;

    private ArmPresets(double length, double angle) {
        this.length = length;
        this.angle = angle;
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
        return angle;
    }

}
