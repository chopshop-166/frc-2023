package frc.robot;

public enum EnumLevel {
    // 0 in.
    LOW_SCORE(0, 0),
    // 34 in.
    MEDIUM_SCORE(3.41, 76.1),
    // 41 7/8 in.
    HIGH_SCORE(18.59, 90),

    FLOOR_PICKUP(0, 0),

    // Human Player Station = HPS
    HPS_PICKUP(0, 76);

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
