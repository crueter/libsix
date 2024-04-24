// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.motor.requests.position;

import frc.lib.beaklib.motor.BeakMotorController;
import frc.lib.beaklib.motor.requests.BeakControlRequest;

/** Drive to a specified position. */
public class BeakPositionNU extends BeakControlRequest {
    /**
     * Position setpoint in NU.
     */
    public double Position = 0.0;

    /**
     * Whether or not to use FOC commutation if supported.
     */
    public boolean UseFOC = false;

    public BeakPositionNU() {
    }

    public void apply(BeakMotorController controller) {
        controller.useFOC(UseFOC);
        controller.setPositionNU(Position);
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param position The new position to use, in NU.
     * @return Itself, with this parameter changed.
     */
    public BeakPositionNU withPosition(double position) {
        this.Position = position;
        return this;
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param useFOC Whether or not to use FOC.
     * @return Itself, with this parameter changed.
     */
    public BeakPositionNU withUseFOC(boolean useFOC) {
        this.UseFOC = useFOC;
        return this;
    }
}
