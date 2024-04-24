// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.motor.requests;

import frc.lib.beaklib.motor.BeakMotorController;

/** Drive based on a voltage. */
public class BeakVoltage extends BeakControlRequest {
    /**
     * Voltage to drive at.
     */
    public double Voltage = 0.0;

    /**
     * Whether or not to use FOC commutation if supported.
     */
    public boolean UseFOC = false;

    public BeakVoltage() {
    }

    public void apply(BeakMotorController controller) {
        controller.useFOC(UseFOC);
        controller.setVoltage(Voltage);
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param volts The new voltage to use.
     * @return Itself, with this parameter changed.
     */
    public BeakVoltage withVoltage(double volts) {
        this.Voltage = volts;
        return this;
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param useFOC Whether or not to use FOC.
     * @return Itself, with this parameter changed.
     */
    public BeakVoltage withUseFOC(boolean useFOC) {
        this.UseFOC = useFOC;
        return this;
    }
}
