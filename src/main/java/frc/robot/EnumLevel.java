package frc.robot;

public enum EnumLevel {
    // 0 in.
    LOW_SCORE(0, 0),
    // 34 in.
    MEDIUM_SCORE(3.75, 78),
    // 41 7/8 in.
    HIGH_SCORE(19.6, 92),

    FLOOR_PICKUP(0, 0),

    // Human Player Station = HPS
    HPS_PICKUP(0, 76),

    ARM_STOWED(0, 0);

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
