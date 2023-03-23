package frc.robot.auto;

public enum CommunityPosition {
    POSITION_123(AutoPath.INTO_COMMUNITY_1_2_3, AutoPath.OUT_OF_COMMUNITY_1_2_3),
    POSITION_456(AutoPath.INTO_COMMUNITY_4_5_6, AutoPath.OUT_OF_COMMUNITY_4_5_6);

    public final AutoPath outOfCommunity;
    public final AutoPath inCommunity;

    private CommunityPosition(AutoPath outOfCommunity, AutoPath inCommunity) {
        this.outOfCommunity = outOfCommunity;
        this.inCommunity = inCommunity;
    }
}
