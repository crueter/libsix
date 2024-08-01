// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.six.motor.requests;

import frc.lib.six.motor.SixMotorController;

/** Drive based on a current. */
public class SixCurrent extends SixControlRequest {
    /**
     * Current to drive at.
     */
    public double Current = 0.0;

    /**
     * Whether or not to use FOC commutation if supported.
     */
    public boolean UseFOC = false;

    public SixCurrent() {
    }

    public void apply(SixMotorController controller) {
        controller.useFOC(UseFOC);
        controller.setCurrent(Current);
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param current The new duty current to use.
     * @return Itself, with this parameter changed.
     */
    public SixCurrent withCurrent(double current) {
        this.Current = current;
        return this;
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param useFOC Whether or not to use FOC.
     * @return Itself, with this parameter changed.
     */
    public SixCurrent withUseFOC(boolean useFOC) {
        this.UseFOC = useFOC;
        return this;
    }
}
