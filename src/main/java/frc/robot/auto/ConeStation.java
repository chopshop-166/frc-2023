package frc.robot.auto;

public enum ConeStation {
    STATION_1(1, AutoPath.UP_TO_CONE_STATION_1, AutoPath.BACKED_UP_1, AutoPath.OUT_OF_COMMUNITY_1_2_3,
            AutoPath.INTO_COMMUNITY_1_2_3),
    STATION_2(2, AutoPath.UP_TO_CONE_STATION_2, AutoPath.BACKED_UP_2, AutoPath.OUT_OF_COMMUNITY_1_2_3,
            AutoPath.INTO_COMMUNITY_1_2_3),
    STATION_3(3, AutoPath.UP_TO_CONE_STATION_3, AutoPath.BACKED_UP_3, AutoPath.OUT_OF_COMMUNITY_1_2_3,
            AutoPath.INTO_COMMUNITY_1_2_3),
    STATION_4(4, AutoPath.UP_TO_CONE_STATION_4, AutoPath.BACKED_UP_4, AutoPath.OUT_OF_COMMUNITY_1_2_3,
            AutoPath.INTO_COMMUNITY_1_2_3),
    STATION_5(5, AutoPath.UP_TO_CONE_STATION_5, AutoPath.BACKED_UP_5, AutoPath.OUT_OF_COMMUNITY_1_2_3,
            AutoPath.INTO_COMMUNITY_1_2_3),
    STATION_6(6, AutoPath.UP_TO_CONE_STATION_6, AutoPath.BACKED_UP_6, AutoPath.OUT_OF_COMMUNITY_1_2_3,
            AutoPath.INTO_COMMUNITY_1_2_3);

    public final int number;
    public final AutoPath upToStation;
    public final AutoPath backedUp;
    public final AutoPath outOfCommunity;
    public final AutoPath inCommunity;

    private ConeStation(int number, AutoPath upToStation, AutoPath backedUp, AutoPath outOfCommunity,
            AutoPath inCommunity) {
        this.number = number;
        this.upToStation = upToStation;
        this.backedUp = backedUp;
        this.outOfCommunity = outOfCommunity;
        this.inCommunity = inCommunity;
    }
}
