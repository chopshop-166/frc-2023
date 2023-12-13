package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.ArmExtendMap;
import frc.robot.maps.subsystems.ArmExtendMap.Data;
import frc.robot.ArmPresets;

public class ArmExtend extends SmartSubsystemBase {

    public Data data = new Data();
    public ArmExtendMap extendMap;
    public final double SPEED = 0.4;
    private final double RETRACT_SPEED = -0.1;
    private final double INTAKE_DEPTH_LIMIT = 1;
    private double armAngle;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    DoublePublisher lengthPub = inst.getDoubleTopic("Arm/Length").publish();
    DoubleSubscriber angleSub = inst.getDoubleTopic("Arm/Angle").subscribe(0);
    BooleanPublisher intakeBelowGroundPublish = inst.getBooleanTopic("Arm/Below Ground").publish();

    public ArmExtend(ArmExtendMap extendMap) {
        this.extendMap = extendMap;
    }

    public Command resetZero(DoubleSupplier speed) {
        return run(() -> {
            data.setPoint = speed.getAsDouble() * SPEED;
        }).finallyDo(() -> {
            extendMap.extendMotor.getEncoder().reset();
        });
    }

    public boolean intakeBelowGround() {
        return extendMap.pivotHeight - INTAKE_DEPTH_LIMIT < Math.cos(Math.toRadians(armAngle))
                * (data.distanceInches + 42.3);
    }

    public Command retract(double speed) {
        return cmd().onInitialize(() -> {
            data.setPoint = -speed;
        }).runsUntil(() -> data.distanceInches < 2.0).onEnd(() -> {
            data.setPoint = 0.0;
        });
    }

    // Manually sets the arm extension
    public Command manual(DoubleSupplier motorSpeed) {
        return run(() -> {
            data.setPoint = limit(motorSpeed.getAsDouble() * SPEED);
        });
    }

    // Uses PID to change extension of arm to set distance
    public Command moveToDistancePID(double distance) {
        return cmd("Move Distance").onInitialize(() -> {
            extendMap.pid.reset(data.distanceInches);
        }).onExecute(() -> {
            // Extend
            data.setPoint = extendMap.pid.calculate(data.distanceInches, distance);
            if (data.setPoint > 0.05) {
                data.setPoint += 0.1;
            }
            if (data.setPoint < -0.05) {
                data.setPoint -= 0.1;
            }

        }).runsUntil(() -> extendMap.pid.atGoal()).onEnd(() -> {
            data.setPoint = 0;
        });
    }

    public Command moveTo(ArmPresets level) {
        return moveToDistancePID(level.getLength());
    }

    // This ensures that the arm is fully retracted (likely for the start or end of
    // a match)
    public Command zeroVelocityCheck() {
        PersistenceCheck velocityPersistenceCheck = new PersistenceCheck(5,
                () -> Math.abs(data.velocityInchesPerSec) < 0.5);
        return cmd("Check Velocity").onInitialize(() -> {
            velocityPersistenceCheck.reset();
            data.setPoint = (RETRACT_SPEED);
        }).runsUntil(velocityPersistenceCheck).onEnd(() -> {
            data.setPoint = 0;
            extendMap.extendMotor.getEncoder().reset();
        });
    }

    @Override
    public void reset() {
        // Nothing to reset here
        extendMap.extendMotor.getEncoder().reset();
    }

    @Override
    public void safeState() {
        // Sets arm extension point to 0
        data.setPoint = 0;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Use this for any background processing
        this.extendMap.updateData(data);
        Logger.processInputs(getName(), data);
        lengthPub.set(data.distanceInches);
        armAngle = angleSub.get();
        intakeBelowGroundPublish.set(intakeBelowGround());
    }

    // Adds limits to arm extension speed

    private double limit(double speed) {

        if (speed > 0 && intakeBelowGround()) {
            return 0;
        }
        if (armAngle < 18 && speed > 0) {
            return 0;
        }

        if ((data.distanceInches > extendMap.hardMaxDistance && speed > 0)
                || (data.distanceInches < extendMap.hardMinDistance && speed < 0)) {
            return data.setPoint = 0;
        }
        if ((data.distanceInches > extendMap.softMaxDistance && speed > 0)
                || (data.distanceInches < extendMap.softMinDistance && speed < 0)) {
            return speed;
        }
        return speed;
    }
}