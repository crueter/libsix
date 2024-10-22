// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.six.gyro;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.lib.six.motor.DataSignal;

/** A Kauai Labs NavX IMU implemented as a SixGyro. */
public class SixNavX extends AHRS implements SixGyro {
    public SixNavX(Port port) {
        super(port);
    }

    @Override
    public DataSignal<Rotation2d> getPitchRotation2d(boolean latencyCompensated) {
        return new DataSignal<Rotation2d>(() -> Rotation2d.fromDegrees(getPitch()));
    }

    @Override
    public DataSignal<Rotation2d> getRollRotation2d(boolean latencyCompensated) {
        return new DataSignal<Rotation2d>(() -> Rotation2d.fromDegrees(getRoll()));
    }

    @Override
    public DataSignal<Rotation2d> getYawRotation2d(boolean latencyCompensated) {
        return new DataSignal<Rotation2d>(() -> getRotation2d());
    }

    @Override
    public DataSignal<Measure<Velocity<Angle>>> getAngularVelocity() {
        return new DataSignal<Measure<Velocity<Angle>>>(() -> DegreesPerSecond.of(getRate()));
    }
}
