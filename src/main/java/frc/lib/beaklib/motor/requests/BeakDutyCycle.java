// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.motor.requests;

import frc.lib.beaklib.motor.BeakMotorController;

/** Drive based on a duty cycle. */
public class BeakDutyCycle extends BeakControlRequest {
    /**
     * Percent to drive at.
     */
    public double Output = 0.0;

    /**
     * Whether or not to use FOC commutation if supported.
     */
    public boolean UseFOC = false;

    public BeakDutyCycle() {
    }

    public void apply(BeakMotorController controller) {
        controller.useFOC(UseFOC);
        controller.set(Output);
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param output The new duty cycle to use.
     * @return Itself, with this parameter changed.
     */
    public BeakDutyCycle withOutput(double output) {
        this.Output = output;
        return this;
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param useFOC Whether or not to use FOC.
     * @return Itself, with this parameter changed.
     */
    public BeakDutyCycle withUseFOC(boolean useFOC) {
        this.UseFOC = useFOC;
        return this;
    }
}
