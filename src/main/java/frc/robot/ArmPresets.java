package frc.robot;

public enum ArmPresets {
    // Need to measure values
    MEDIUM_SCORE(3.8, 78),

    HIGH_SCORE(18.5, 92),

    MEDIUM_SCORE_ACTUAL(3.8, 67),

    HIGH_SCORE_ACTUAL(18.5, 80),

    // Human Player Station = HPS
    HPS_PICKUP(0, 76),

    ARM_STOWED(1, 4),

    CUBE_PICKUP(1, 4);

    private double length;
    private double angle;

    private ArmPresets(double length, double angle) {
        this.length = length;
        this.angle = angle;
    }

    public double getLength() {
        return length;
    }

    public double getAngle() {
        return angle;
    }

}
