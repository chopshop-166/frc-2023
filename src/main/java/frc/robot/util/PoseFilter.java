package frc.robot.util;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PoseFilter {

    private LinearFilter xFilter;
    private LinearFilter yFilter;
    private LinearFilter angleFilter;

    public PoseFilter(double timeConstant, double period) {
        xFilter = LinearFilter.singlePoleIIR(timeConstant, period);
        yFilter = LinearFilter.singlePoleIIR(timeConstant, period);
        angleFilter = LinearFilter.singlePoleIIR(timeConstant, period);
    }

    public Pose2d calculate(Pose2d input) {
        return new Pose2d(
                xFilter.calculate(input.getX()),
                yFilter.calculate(input.getY()),
                Rotation2d.fromDegrees(angleFilter.calculate(input.getRotation().getDegrees())));
    }

}
