// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.motor.configs;

/** Limit switch configurations. */
public class BeakHardwareLimitSwitchConfigs {
    public enum BeakLimitSwitchSource {
        None,
        Connected,
        DIO
    }

    /**
     * If using DIO/remote limit switch, specifies the ID to poll.
     */
    public int ForwardLimitSwitchID = 0;
    
    /**
     * The source of the forward limit switch (connected to data port or DIO)
     */
    public BeakLimitSwitchSource ForwardSource = BeakLimitSwitchSource.None;

    /**
     * Whether or not the forward limit switch is set to normally-closed.
     */
    public boolean ForwardNormallyClosed = true;

    /**
     * If using DIO/remote limit switch, specifies the ID to poll.
     */
    public int ReverseLimitSwitchID = 0;
    
    /**
     * The source of the reverse limit switch (connected to data port or DIO)
     */
    public BeakLimitSwitchSource ReverseSource = BeakLimitSwitchSource.None;

    /**
     * Whether or not the reverse limit switch is set to normally-closed.
     */
    public boolean ReverseNormallyClosed = true;

    public BeakHardwareLimitSwitchConfigs() {
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param forwardLimitSwitchID The forward limit switch remote ID.
     * @return Itself, with this parameter changed.
     */
    public BeakHardwareLimitSwitchConfigs withForwardLimitSwitchID(int forwardLimitSwitchID) {
        ForwardLimitSwitchID = forwardLimitSwitchID;
        return this;
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param forwardSource The forward limit switch source.
     * @return Itself, with this parameter changed.
     */
    public BeakHardwareLimitSwitchConfigs withForwardSource(BeakLimitSwitchSource forwardSource) {
        ForwardSource = forwardSource;
        return this;
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param forwardNormallyClosed If the forward limit switch is normally closed.
     * @return Itself, with this parameter changed.
     */
    public BeakHardwareLimitSwitchConfigs withForwardNormallyClosed(boolean forwardNormallyClosed) {
        ForwardNormallyClosed = forwardNormallyClosed;
        return this;
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param reverseLimitSwitchID The reverse limit switch remote ID.
     * @return Itself, with this parameter changed.
     */
    public BeakHardwareLimitSwitchConfigs withReverseLimitSwitchID(int reverseLimitSwitchID) {
        ReverseLimitSwitchID = reverseLimitSwitchID;
        return this;
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param reverseSource The reverse limit switch source.
     * @return Itself, with this parameter changed.
     */
    public BeakHardwareLimitSwitchConfigs withReverseSource(BeakLimitSwitchSource reverseSource) {
        ReverseSource = reverseSource;
        return this;
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param reverseNormallyClosed If the reverse limit switch is normally closed.
     * @return Itself, with this parameter changed.
     */
    public BeakHardwareLimitSwitchConfigs withReverseNormallyClosed(boolean reverseNormallyClosed) {
        ReverseNormallyClosed = reverseNormallyClosed;
        return this;
    }
}
