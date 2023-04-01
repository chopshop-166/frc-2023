package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.pneumatics.IDSolenoid;
import com.chopshop166.chopshoplib.pneumatics.MockDSolenoid;
import com.chopshop166.chopshoplib.pneumatics.MockSolenoid;

public class BalanceArmMap {

    private final IDSolenoid solenoid;

    public BalanceArmMap() {
        this(new MockDSolenoid());
    }

    public BalanceArmMap(IDSolenoid solenoid) {
        this.solenoid = solenoid;
    }

    public IDSolenoid getSolenoid() {
        return solenoid;
    }
}
