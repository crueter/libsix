// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.six.motor.requests.velocity;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.lib.six.motor.SixMotorController;
import frc.lib.six.motor.requests.SixControlRequest;

/** Drive to a specified velocity target. */
public class SixVelocity extends SixControlRequest {
    /**
     * Velocity to drive at.
     */
    public Measure<Velocity<Distance>> Velocity = MetersPerSecond.zero();

    /**
     * Whether or not to use FOC commutation if supported.
     */
    public boolean UseFOC = false;

    /**
     * Output value (current, voltage, duty cycle)
     */
    public OutputType Output = OutputType.Voltage;

    public SixVelocity() {
    }

    public void apply(SixMotorController controller) {
        controller.setNextOutputType(Output);
        controller.useFOC(UseFOC);
        controller.setVelocity(Velocity);
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param velocity The new velocity to use.
     * @return Itself, with this parameter changed.
     */
    public SixVelocity withVelocity(Measure<Velocity<Distance>> velocity) {
        this.Velocity = velocity;
        return this;
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param useFOC Whether or not to use FOC.
     * @return Itself, with this parameter changed.
     */
    public SixVelocity withUseFOC(boolean useFOC) {
        this.UseFOC = useFOC;
        return this;
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param type What output type to use: voltage, duty cycle, current
     * @return Itself, with this parameter changed.
     */
    public SixVelocity withOutputType(OutputType type) {
        this.Output = type;
        return this;
    }

}
