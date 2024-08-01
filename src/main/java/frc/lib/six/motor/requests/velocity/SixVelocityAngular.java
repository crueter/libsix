// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.six.motor.requests.velocity;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.lib.six.motor.SixMotorController;
import frc.lib.six.motor.requests.SixControlRequest;

/** Drive to a specified angular velocity target. */
public class SixVelocityAngular extends SixControlRequest {
    /**
     * Angular Velocity to drive at.
     */
    public Measure<Velocity<Angle>> Velocity = RPM.zero();

    /**
     * Whether or not to use FOC commutation if supported.
     */
    public boolean UseFOC = false;

    /**
     * Output value (current, voltage, duty cycle)
     */
    public OutputType Output = OutputType.Voltage;

    public SixVelocityAngular() {
    }

    public void apply(SixMotorController controller) {
        controller.setNextOutputType(Output);
        controller.useFOC(UseFOC);
        controller.setAngularVelocity(Velocity);
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param velocity The new velocity to use.
     * @return Itself, with this parameter changed.
     */
    public SixVelocityAngular withVelocity(Measure<Velocity<Angle>> velocity) {
        this.Velocity = velocity;
        return this;
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param useFOC Whether or not to use FOC.
     * @return Itself, with this parameter changed.
     */
    public SixVelocityAngular withUseFOC(boolean useFOC) {
        this.UseFOC = useFOC;
        return this;
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param type What output type to use: voltage, duty cycle, current
     * @return Itself, with this parameter changed.
     */
    public SixVelocityAngular withOutputType(OutputType type) {
        this.Output = type;
        return this;
    }

}
