package frc.robot;

public enum EnumLevel {
    // 0 in.
    LOW(0, 0),
    // 34 in.
    MEDIUM(0, 0),
    // 41 7/8 in.
    HIGH(0, 0);

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
