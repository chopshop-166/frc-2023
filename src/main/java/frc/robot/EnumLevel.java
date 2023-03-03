package frc.robot;

public enum EnumLevel {
    // Need to measure values
    LOW_SCORE(0, 0),
    //
    MEDIUM_SCORE(3.75, 78),
    //
    HIGH_SCORE(17, 92),

    // Need to measure values 
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
