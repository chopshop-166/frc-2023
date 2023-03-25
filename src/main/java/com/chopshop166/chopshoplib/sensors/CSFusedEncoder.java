package com.chopshop166.chopshoplib.sensors;

import edu.wpi.first.util.sendable.SendableBuilder;

public class CSFusedEncoder implements IEncoder {

    IEncoder relativeEncoder;
    IAbsolutePosition absolutePos;

    double relativeEncoderOffset;

    public CSFusedEncoder(IEncoder relativeEncoder, IAbsolutePosition absPosition) {
        this.relativeEncoder = relativeEncoder;
        this.absolutePos = absPosition;
        relativeEncoderOffset = this.absolutePos.getAbsolutePosition();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Quadrature Encoder");
        builder.addDoubleProperty("Speed", this::getRate, null);
        builder.addDoubleProperty("Distance", this::getDistance, null);
    }

    @Override
    public void reset() {
        // This sets the relative encoder to the current absolute position
        relativeEncoderOffset = this.absolutePos.getAbsolutePosition();
        this.relativeEncoder.reset();
    }

    @Override
    public double getDistance() {
        return this.relativeEncoder.getDistance() + relativeEncoderOffset;
    }

    @Override
    public double getRate() {
        return this.relativeEncoder.getRate();
    }

    @Override
    public double getAbsolutePosition() {
        return this.absolutePos.getAbsolutePosition();
    }

}
