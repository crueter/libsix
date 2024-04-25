// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.motor.configs;

/** Configurations for duty cycle output modes. */
public class BeakDutyCycleConfigs {
    /**
     * The percent at which it will be considered zero output.
     */
    public double NeutralDeadband = 0.05;

    /**
     * The peak output in the positive direction.
     */
    public double PeakForwardOutput = 1.0;

    /**
     * The peak output in the negative direction.
     */
    public double PeakReverseOutput = -1.0;

    /**
     * The time it takes to go from 0 to 100% in duty cycle closed-loop modes.
     */
    public double ClosedRampPeriod = 0.0;

    /**
     * The time it takes to go from 0 to 100% in duty cycle open-loop modes.
     */
    public double OpenRampPeriod = 0.0;

    public BeakDutyCycleConfigs() {
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param neutralDeadband The deadband below which a command is considered
     *                        neutral output.
     * @return Itself, with this parameter changed.
     */
    public BeakDutyCycleConfigs withNeutralDeadband(double neutralDeadband) {
        NeutralDeadband = neutralDeadband;
        return this;
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param peakForwardOutput The highest positive output allowed.
     * @return Itself, with this parameter changed.
     */
    public BeakDutyCycleConfigs withPeakForwardOutput(double peakForwardOutput) {
        PeakForwardOutput = peakForwardOutput;
        return this;
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param peakReverseOutput The highest negative output allowed.
     * @return Itself, with this parameter changed.
     */
    public BeakDutyCycleConfigs withPeakReverseOutput(double peakReverseOutput) {
        PeakReverseOutput = peakReverseOutput;
        return this;
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param closedRampPeriod The time to go from 0 to 100% in closed loop modes.
     * @return Itself, with this parameter changed.
     */
    public BeakDutyCycleConfigs withClosedRampPeriod(double closedRampPeriod) {
        ClosedRampPeriod = closedRampPeriod;
        return this;
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param openRampPeriod The time to go from 0 to 100% in open loop modes.
     * @return Itself, with this parameter changed.
     */
    public BeakDutyCycleConfigs withOpenRampPeriod(double openRampPeriod) {
        OpenRampPeriod = openRampPeriod;
        return this;
    }
}
