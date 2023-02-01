package frc.robot.maps;

import frc.robot.maps.subsystems.LedMap;

public class Valkryie extends RobotMap {
    @Override
    public LedMap getLedMap() {
        return new LedMap(0, 30);
    }

}
