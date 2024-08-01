// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.six.motor.configs;

import edu.wpi.first.math.geometry.Rotation2d;

/** Software limit switch configurations. */
public class SixSoftLimitConfigs {

    /**
     * The number of motor rotations to be considered the forward extreme. (Must be positive)
     */
    public Rotation2d ForwardLimit = new Rotation2d();

    /**
     * The number of motor rotations to be considered the reverse extreme. (Must be negative)
     */
    public Rotation2d ReverseLimit = new Rotation2d();

    public SixSoftLimitConfigs() {
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param limit Output shaft rotations to be considered the forward extreme.
     * @return Itself, with this parameter changed.
     */
    public SixSoftLimitConfigs withForwardLimit(Rotation2d limit) {
        this.ForwardLimit = limit;
        return this;
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param limit Output shaft rotations to be considered the reverse extreme.
     * @return Itself, with this parameter changed.
     */
    public SixSoftLimitConfigs withReverseLimit(Rotation2d limit) {
        this.ReverseLimit = limit;
        return this;
    }
}
