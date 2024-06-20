// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.motor.requests;

import frc.lib.beaklib.motor.BeakMotorController;

/** Base class for motor control requests. */
public class BeakControlRequest {
    public enum OutputType {
        Voltage,
        DutyCycle,
        Current
    }

    public void apply(BeakMotorController controller) {}
}
