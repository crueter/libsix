// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.six.encoder;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.lib.six.motor.DataSignal;

/** SixLib implementation of the REV Absolute Encoder. */
public class SixSparkMAXEncoder implements SixAbsoluteEncoder {
    SparkAbsoluteEncoder m_encoder;

    public SixSparkMAXEncoder(SparkAbsoluteEncoder encoder) {
        m_encoder = encoder;
    }
    @Override
    public DataSignal<Rotation2d> getAbsoluteEncoderPosition(boolean latencyCompensated) {
        // SUSSY
        return new DataSignal<Rotation2d>(
                () -> Rotation2d.fromRotations(m_encoder.getPosition()));
    }

    @Override
    public DataSignal<Rotation2d> getEncoderPosition(boolean latencyCompensated) {
        // SUSSY
        return new DataSignal<Rotation2d>(
                () -> Rotation2d.fromRotations(m_encoder.getPosition()));
    }

    @Override
    public void setEncoderPosition(Rotation2d position) {
    }

    @Override
    public DataSignal<Measure<Velocity<Angle>>> getEncoderVelocity() {
        return new DataSignal<Measure<Velocity<Angle>>>(() -> RotationsPerSecond.of(m_encoder.getVelocity()));
    }

    @Override
    public void setAbsoluteOffset(Rotation2d offset) {
        m_encoder.setZeroOffset(offset.getRotations());
    }

    @Override
    public Rotation2d getAbsoluteOffset() {
        return Rotation2d.fromRotations(m_encoder.getZeroOffset());
    }

    @Override
    public void setDataFramePeriod(int period) {
    }

    @Override
    public void setCWPositive(boolean cwPositive) {
        m_encoder.setInverted(cwPositive);
    }

    @Override
    public void restoreFactoryDefault() {
    }
}
