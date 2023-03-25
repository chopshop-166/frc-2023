package frc.robot.auto;

public enum ConeStation {
    STATION_1(1, AutoPath.UP_TO_CONE_STATION_1, AutoPath.BACKED_UP_1, CommunityPosition.POSITION_123),
    STATION_2(2, AutoPath.UP_TO_CONE_STATION_2, AutoPath.BACKED_UP_2, CommunityPosition.POSITION_123),
    STATION_3(3, AutoPath.UP_TO_CONE_STATION_3, AutoPath.BACKED_UP_3, CommunityPosition.POSITION_123),
    STATION_4(4, AutoPath.UP_TO_CONE_STATION_4, AutoPath.BACKED_UP_4, CommunityPosition.POSITION_456),
    STATION_5(5, AutoPath.UP_TO_CONE_STATION_5, AutoPath.BACKED_UP_5, CommunityPosition.POSITION_456),
    STATION_6(6, AutoPath.UP_TO_CONE_STATION_6, AutoPath.BACKED_UP_6, CommunityPosition.POSITION_456);

    public final int number;
    public final AutoPath upToStation;
    public final AutoPath backedUp;
    public final CommunityPosition communityPosition;

    private ConeStation(int number, AutoPath upToStation, AutoPath backedUp,
            CommunityPosition communityPosition) {
        this.number = number;
        this.upToStation = upToStation;
        this.backedUp = backedUp;
        this.communityPosition = communityPosition;
    }
}
