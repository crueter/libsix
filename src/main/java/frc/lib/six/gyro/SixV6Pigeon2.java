// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.six.gyro;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.lib.six.motor.DataSignal;

/** A v6 CTRE Pigeon 2 implemented as a SixGyro. */
public class SixV6Pigeon2 extends Pigeon2 implements SixGyro {
    public SixV6Pigeon2(int port) {
        super(port);
    }

    public SixV6Pigeon2(int port, String canBus) {
        super(port, canBus);
    }

    @Override
    public DataSignal<Rotation2d> getPitchRotation2d(boolean latencyCompensated) {
        StatusSignal<Double> pitch = getPitch();

        return new DataSignal<Rotation2d>(
                () -> {
                    double pitchValue;

                    if (latencyCompensated) {
                        // this isn't documented well, but:
                        // pitch is about Y
                        // roll is about X
                        // yaw is about Z
                        pitchValue = StatusSignal.getLatencyCompensatedValue(pitch, getAngularVelocityYDevice());
                    } else {
                        pitchValue = pitch.getValue();
                    }

                    return Rotation2d.fromDegrees(pitchValue);
                },
                () -> pitch.getTimestamp().getTime(),
                pitch::refresh,
                pitch::setUpdateFrequency);
    }

    @Override
    public DataSignal<Rotation2d> getRollRotation2d(boolean latencyCompensated) {
        StatusSignal<Double> roll = getRoll();

        return new DataSignal<Rotation2d>(
                () -> {
                    double rollValue;

                    if (latencyCompensated) {
                        // this isn't documented well, but:
                        // pitch is about Y
                        // roll is about X
                        // yaw is about Z
                        rollValue = StatusSignal.getLatencyCompensatedValue(roll, getAngularVelocityXDevice());
                    } else {
                        rollValue = roll.getValue();
                    }

                    return Rotation2d.fromDegrees(rollValue);
                },
                () -> roll.getTimestamp().getTime(),
                roll::refresh,
                roll::setUpdateFrequency);
    }

    @Override
    public DataSignal<Rotation2d> getYawRotation2d(boolean latencyCompensated) {
        StatusSignal<Double> yaw = getYaw();

        return new DataSignal<Rotation2d>(
                () -> {
                    double yawValue;

                    if (latencyCompensated) {
                        // this isn't documented well, but:
                        // pitch is about Y
                        // roll is about X
                        // yaw is about Z
                        yawValue = StatusSignal.getLatencyCompensatedValue(yaw, getAngularVelocityZDevice());
                    } else {
                        yawValue = yaw.getValue();
                    }

                    return Rotation2d.fromDegrees(yawValue);
                },
                () -> yaw.getTimestamp().getTime(),
                yaw::refresh,
                yaw::setUpdateFrequency);
    }

    @Override
    public DataSignal<Measure<Velocity<Angle>>> getAngularVelocity() {
        StatusSignal<Double> angularVelocity = super.getAngularVelocityZDevice();

        return new DataSignal<Measure<Velocity<Angle>>>(
                () -> DegreesPerSecond.of(angularVelocity.getValue()),
                () -> angularVelocity.getTimestamp().getTime(),
                angularVelocity::refresh,
                angularVelocity::setUpdateFrequency);
    }

}
