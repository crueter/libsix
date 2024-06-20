// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.encoder;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.lib.beaklib.motor.DataSignal;

/** BeakLib implementation of the CTRE CANCoder. */
public class BeakCANCoder extends CANcoder implements BeakAbsoluteEncoder {
    CANcoderConfigurator m_configurator;

    public BeakCANCoder(int port) {
        this(port, "");
    }

    public BeakCANCoder(int port, String canBus) {
        super(port, canBus);

        m_configurator = super.getConfigurator();
    }

    @Override
    public DataSignal<Rotation2d> getEncoderPosition(boolean latencyCompensated) {
        StatusSignal<Double> position = getPosition();

        return new DataSignal<Rotation2d>(() -> {
            double positionValue;

            if (latencyCompensated) {
                positionValue = StatusSignal.getLatencyCompensatedValue(position, getVelocity());
            } else {
                positionValue = position.getValue();
            }

            return new Rotation2d(positionValue * 2 * Math.PI);
        },
                () -> position.getTimestamp().getTime(),
                position::refresh,
                position::setUpdateFrequency);
    }

    @Override
    public void setEncoderPosition(Rotation2d position) {
        super.setPosition(position.getRadians() / (2 * Math.PI));
    }

    @Override
    public DataSignal<Measure<Velocity<Angle>>> getEncoderVelocity() {
        StatusSignal<Double> velocity = getVelocity();

        return new DataSignal<Measure<Velocity<Angle>>>(() -> RotationsPerSecond.of(velocity.getValue()),
                () -> velocity.getTimestamp().getTime(), velocity::refresh, velocity::setUpdateFrequency);
    }

    @Override
    public void setAbsoluteOffset(Rotation2d offset) {
        MagnetSensorConfigs config = new MagnetSensorConfigs();
        m_configurator.refresh(config);

        config.MagnetOffset = offset.getRadians() / (2 * Math.PI);

        m_configurator.apply(config);
    }

    @Override
    public Rotation2d getAbsoluteOffset() {
        MagnetSensorConfigs config = new MagnetSensorConfigs();
        m_configurator.refresh(config);

        return new Rotation2d(config.MagnetOffset * 2 * Math.PI);
    }

    @Override
    public void setDataFramePeriod(int period) {
        getPosition().setUpdateFrequency(1 / (period * 1000));
        getAbsolutePosition().setUpdateFrequency(1 / (period * 1000));
        getVelocity().setUpdateFrequency(1 / (period * 1000));
    }

    @Override
    public void setCWPositive(boolean cwPositive) {
        MagnetSensorConfigs config = new MagnetSensorConfigs();
        m_configurator.refresh(config);

        config.SensorDirection = cwPositive ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;

        m_configurator.apply(config);
    }

    @Override
    public void restoreFactoryDefault() {
        m_configurator.apply(new CANcoderConfiguration());
    }

    @Override
    public DataSignal<Rotation2d> getAbsoluteEncoderPosition(boolean latencyCompensated) {
        StatusSignal<Double> position = getAbsolutePosition();

        return new DataSignal<Rotation2d>(() -> {
            double positionValue;

            if (latencyCompensated) {
                positionValue = StatusSignal.getLatencyCompensatedValue(position, getVelocity());
            } else {
                positionValue = position.getValue();
            }

            return new Rotation2d(positionValue * 2 * Math.PI);
        },
                () -> position.getTimestamp().getTime(),
                position::refresh,
                position::setUpdateFrequency);
    }
}
