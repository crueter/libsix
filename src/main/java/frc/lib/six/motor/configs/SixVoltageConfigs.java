// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.six.motor.configs;

/** Configurations for voltage output modes. */
public class SixVoltageConfigs {
    /**
     * The percent at which it will be considered zero output.
     */
    public double NeutralDeadband = 0.25;

    /**
     * The peak output in the positive direction.
     */
    public double PeakForwardOutput = 12.0;

    /**
     * The peak output in the negative direction.
     */
    public double PeakReverseOutput = -12.0;

    /**
     * The time it takes to go from 0 to 12V in voltage closed-loop modes.
     */
    public double ClosedRampPeriod = 0.0;

    /**
     * The time it takes to go from 0 to 12V in voltage open-loop modes.
     */
    public double OpenRampPeriod = 0.0;

    /**
     * The nominal supply voltage of the motor controller (usually 12V).
     */
    public double NominalVoltage = 12.0;

    public SixVoltageConfigs() {
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param neutralDeadband The deadband below which a command is considered
     *                        neutral output.
     * @return Itself, with this parameter changed.
     */
    public SixVoltageConfigs withNeutralDeadband(double neutralDeadband) {
        NeutralDeadband = neutralDeadband;
        return this;
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param peakForwardOutput The highest positive output allowed.
     * @return Itself, with this parameter changed.
     */
    public SixVoltageConfigs withPeakForwardOutput(double peakForwardOutput) {
        PeakForwardOutput = peakForwardOutput;
        return this;
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param peakReverseOutput The highest negative output allowed.
     * @return Itself, with this parameter changed.
     */
    public SixVoltageConfigs withPeakReverseOutput(double peakReverseOutput) {
        PeakReverseOutput = peakReverseOutput;
        return this;
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param closedRampPeriod The time to go from 0 to 12V in closed loop modes.
     * @return Itself, with this parameter changed.
     */
    public SixVoltageConfigs withClosedRampPeriod(double closedRampPeriod) {
        ClosedRampPeriod = closedRampPeriod;
        return this;
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param openRampPeriod The time to go from 0 to 12V in open loop modes.
     * @return Itself, with this parameter changed.
     */
    public SixVoltageConfigs withOpenRampPeriod(double openRampPeriod) {
        OpenRampPeriod = openRampPeriod;
        return this;
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param nominalVoltage The nominal supply voltage of the motor controller (usually 12V).
     * @return Itself, with this parameter changed.
     */
    public SixVoltageConfigs withNominalVoltage(double nominalVoltage) {
        NominalVoltage = nominalVoltage;
        return this;
    }
}
