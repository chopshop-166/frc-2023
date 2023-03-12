package frc.robot;

public enum EnumLevel {
    // Need to measure values
    LOW_SCORE(0, 0),
    //
    MEDIUM_SCORE(3.8, 78),
    // 41 7/8 in.
    HIGH_SCORE(18.5, 92),

    MEDIUM_SCORE_ACTUAL(3.8, 67),

    HIGH_SCORE_ACTUAL(18.5, 80),

    // Need to measure values
    FLOOR_PICKUP(0, 0),
    // Human Player Station = HPS
    HPS_PICKUP(0, 76),

    ARM_STOWED(1, 4);CUBE_PICKUP(1,4);

    private double length;
    private double angle;

    private EnumLevel(double length, double angle) {
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
