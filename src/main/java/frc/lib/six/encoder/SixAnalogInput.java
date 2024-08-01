// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.six.encoder;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.six.motor.DataSignal;

/** CANCoder, as a {@link SixAbsoluteEncoder}. */
public class SixAnalogInput extends AnalogInput implements SixAbsoluteEncoder {
    private Rotation2d m_offset;

    public SixAnalogInput(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public DataSignal<Rotation2d> getAbsoluteEncoderPosition(boolean latencyCompensated) {
        // SUSSY
        return new DataSignal<Rotation2d>(
                () -> new Rotation2d((1.0 - super.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI
                        + m_offset.getRadians()));
    }

    @Override
    public Rotation2d getAbsoluteOffset() {
        return m_offset;
    }

    @Override
    public DataSignal<Rotation2d> getEncoderPosition(boolean latencyCompensated) {
        return getAbsoluteEncoderPosition(latencyCompensated);
    }

    @Override
    public void setEncoderPosition(Rotation2d position) {
    }

    @Override
    public DataSignal<Measure<Velocity<Angle>>> getEncoderVelocity() {
        return new DataSignal<Measure<Velocity<Angle>>>(() -> RPM.zero());
    }

    @Override
    public void setAbsoluteOffset(Rotation2d offset) {
        m_offset = offset;
    }

    @Override
    public void setDataFramePeriod(int period) {
    }

    @Override
    public void setCWPositive(boolean cwPositive) {
    }

    @Override
    public void restoreFactoryDefault() {
    }

}
