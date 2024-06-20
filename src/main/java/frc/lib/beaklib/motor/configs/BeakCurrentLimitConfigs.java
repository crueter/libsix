// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.motor.configs;

import com.ctre.phoenix6.configs.TorqueCurrentConfigs;

/**
 * Configurations for stator and supply current limits.
 * <p>
 * Note that for TalonFX on TorqueCurrent modes, you must use
 * {@link TorqueCurrentConfigs} instead.
 */
public class BeakCurrentLimitConfigs {
    /**
     * The limit to (attempt to) hold the supply at.
     */
    public double SupplyCurrentLimit = 0.0;

    /**
     * The current threshold at which to start attempting to hold current. No effect
     * on Sparks.
     */
    public double SupplyCurrentThreshold = 0.0;

    /**
     * Allow unlimited current draw for this amount of time before limits kick in.
     */
    public double SupplyTimeThreshold = 0.0;

    /**
     * What to limit the stator current to. Only effective on TalonFX.
     */
    public double StatorCurrentLimit = 0.0;

    public BeakCurrentLimitConfigs() {
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param supplyCurrentLimit The supply-side current limit to enforce.
     * @return Itself, with this parameter changed.
     */
    public BeakCurrentLimitConfigs withSupplyCurrentLimit(double supplyCurrentLimit) {
        SupplyCurrentLimit = supplyCurrentLimit;
        return this;
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param supplyCurrentThreshold The threshold to start enforcing the
     *                               supply-side current limit.
     * @return Itself, with this parameter changed.
     */
    public BeakCurrentLimitConfigs withSupplyCurrentThreshold(double supplyCurrentThreshold) {
        SupplyCurrentThreshold = supplyCurrentThreshold;
        return this;
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param supplyTimeThreshold The time before the supply current limit is enforced.
     * @return Itself, with this parameter changed.
     */
    public BeakCurrentLimitConfigs withSupplyTimeThreshold(double supplyTimeThreshold) {
        SupplyTimeThreshold = supplyTimeThreshold;
        return this;
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param statorCurrentLimit The stator-side current limit to enforce.
     * @return Itself, with this parameter changed.
     */
    public BeakCurrentLimitConfigs withStatorCurrentLimit(double statorCurrentLimit) {
        StatorCurrentLimit = statorCurrentLimit;
        return this;
    }
}
