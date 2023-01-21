package frc.robot.maps;

import com.chopshop166.chopshoplib.motors.CSSparkMax;
import com.chopshop166.chopshoplib.pneumatics.RevDSolenoid;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.maps.subsystems.ArmMap;

public class Valkyrie extends RobotMap {

    @Override
    public ArmMap getArmMap() {
        CSSparkMax motor = new CSSparkMax(9, MotorType.kBrushless);

        return new ArmMap(motor);
    }
}
