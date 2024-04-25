// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.motor.configs;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

/** Configuration for motion profiling (MotionMagic/SmartMotion). */
public class BeakMotionProfileConfigs {
    /**
     * The cruising velocity of the motion profile.
     */
    public Measure<Velocity<Angle>> Velocity = RPM.zero();

    /**
     * The acceleration of the motion profile.
     */
    public Measure<Velocity<Velocity<Angle>>> Acceleration = RPM.per(Second).zero();

    /**
     * The jerk of the motion profile.
     */
    public Measure<Velocity<Velocity<Velocity<Angle>>>> Jerk = RPM.per(Second).per(Second).zero();

    public BeakMotionProfileConfigs() {
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param velocity The velocity of the motion profile.
     * @return Itself, with this parameter changed.
     */
    public BeakMotionProfileConfigs withVelocity(Measure<Velocity<Angle>> velocity) {
        Velocity = velocity;
        return this;
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param acceleration The acceleration of the motion profile.
     * @return Itself, with this parameter changed.
     */
    public BeakMotionProfileConfigs withAcceleration(Measure<Velocity<Velocity<Angle>>> acceleration) {
        Acceleration = acceleration;
        return this;
    }

    /**
     * Method-chaining API for this config.
     * 
     * @param jerk The jerk of the motion profile.
     * @return Itself, with this parameter changed.
     */
    public BeakMotionProfileConfigs withJerk(Measure<Velocity<Velocity<Velocity<Angle>>>> jerk) {
        Jerk = jerk;
        return this;
    }
}
