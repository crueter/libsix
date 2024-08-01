// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.six.motor.requests.velocity;

import frc.lib.six.motor.SixMotorController;
import frc.lib.six.motor.requests.SixControlRequest;

/** Drive to a specified velocity target. */
public class SixVelocityNU extends SixControlRequest {
    /**
     * Velocity to drive at, in NU.
     */
    public double Velocity = 0.0;

    /**
     * Whether or not to use FOC commutation if supported.
     */
    public boolean UseFOC = false;

    /**
     * Output value (current, voltage, duty cycle)
     */
    public OutputType Output = OutputType.Voltage;

    public SixVelocityNU() {
    }

    public void apply(SixMotorController controller) {
        controller.setNextOutputType(Output);
        controller.useFOC(UseFOC);
        controller.setVelocityNU(Velocity);
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param velocity The new velocity to use, in NU.
     * @return Itself, with this parameter changed.
     */
    public SixVelocityNU withVelocity(double velocity) {
        this.Velocity = velocity;
        return this;
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param useFOC Whether or not to use FOC.
     * @return Itself, with this parameter changed.
     */
    public SixVelocityNU withUseFOC(boolean useFOC) {
        this.UseFOC = useFOC;
        return this;
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param type What output type to use: voltage, duty cycle, current
     * @return Itself, with this parameter changed.
     */
    public SixVelocityNU withOutputType(OutputType type) {
        this.Output = type;
        return this;
    }

}
