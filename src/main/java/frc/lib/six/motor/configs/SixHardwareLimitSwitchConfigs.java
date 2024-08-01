// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.six.motor.configs;

/** Limit switch configurations. */
public class SixHardwareLimitSwitchConfigs {
    public enum SixLimitSwitchSource {
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
    public SixLimitSwitchSource ForwardSource = SixLimitSwitchSource.None;

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
    public SixLimitSwitchSource ReverseSource = SixLimitSwitchSource.None;

    /**
     * Whether or not the reverse limit switch is set to normally-closed.
     */
    public boolean ReverseNormallyClosed = true;

    public SixHardwareLimitSwitchConfigs() {
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param forwardLimitSwitchID The forward limit switch remote ID.
     * @return Itself, with this parameter changed.
     */
    public SixHardwareLimitSwitchConfigs withForwardLimitSwitchID(int forwardLimitSwitchID) {
        ForwardLimitSwitchID = forwardLimitSwitchID;
        return this;
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param forwardSource The forward limit switch source.
     * @return Itself, with this parameter changed.
     */
    public SixHardwareLimitSwitchConfigs withForwardSource(SixLimitSwitchSource forwardSource) {
        ForwardSource = forwardSource;
        return this;
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param forwardNormallyClosed If the forward limit switch is normally closed.
     * @return Itself, with this parameter changed.
     */
    public SixHardwareLimitSwitchConfigs withForwardNormallyClosed(boolean forwardNormallyClosed) {
        ForwardNormallyClosed = forwardNormallyClosed;
        return this;
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param reverseLimitSwitchID The reverse limit switch remote ID.
     * @return Itself, with this parameter changed.
     */
    public SixHardwareLimitSwitchConfigs withReverseLimitSwitchID(int reverseLimitSwitchID) {
        ReverseLimitSwitchID = reverseLimitSwitchID;
        return this;
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param reverseSource The reverse limit switch source.
     * @return Itself, with this parameter changed.
     */
    public SixHardwareLimitSwitchConfigs withReverseSource(SixLimitSwitchSource reverseSource) {
        ReverseSource = reverseSource;
        return this;
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param reverseNormallyClosed If the reverse limit switch is normally closed.
     * @return Itself, with this parameter changed.
     */
    public SixHardwareLimitSwitchConfigs withReverseNormallyClosed(boolean reverseNormallyClosed) {
        ReverseNormallyClosed = reverseNormallyClosed;
        return this;
    }
}
