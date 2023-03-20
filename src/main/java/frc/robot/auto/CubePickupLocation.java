package frc.robot.auto;

public enum CubePickupLocation {
    CUBE_1(1, AutoPath.READY_FOR_PICKUP_7, AutoPath.GO_TO_PICKUP_7),
    CUBE_2(2, AutoPath.READY_FOR_PICKUP_8, AutoPath.GO_TO_PICKUP_8),
    CUBE_3(3, AutoPath.READY_FOR_PICKUP_9, AutoPath.GO_TO_PICKUP_9),
    CUBE_4(4, AutoPath.READY_FOR_PICKUP_10, AutoPath.GO_TO_PICKUP_10);

    public final int number;
    public final AutoPath readyForPickup;
    public final AutoPath goToPickup;

    private CubePickupLocation(int number, AutoPath readyForPickup, AutoPath goToPickup) {
        this.number = number;
        this.readyForPickup = readyForPickup;
        this.goToPickup = goToPickup;
    }
}
