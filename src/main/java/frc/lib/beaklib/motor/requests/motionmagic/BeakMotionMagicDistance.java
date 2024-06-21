// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.motor.requests.motionmagic;

import static edu.wpi.first.units.Units.Feet;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import frc.lib.beaklib.motor.BeakMotorController;
import frc.lib.beaklib.motor.requests.BeakControlRequest;

/** Drive to a specified distance using smart motion / motion magic. */
public class BeakMotionMagicDistance extends BeakControlRequest {
    /**
     * Position setpoint.
     */
    public Measure<Distance> Position = Feet.zero();

    /**
     * Whether or not to use FOC commutation if supported.
     */
    public boolean UseFOC = false;

    /**
     * Output value (current, voltage, duty cycle)
     */
    public OutputType Output = OutputType.Voltage;

    public BeakMotionMagicDistance() {
    }

    public void apply(BeakMotorController controller) {
        controller.setNextOutputType(Output);
        controller.useFOC(UseFOC);
        controller.setMotionProfile(Position);
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param position The new position to use.
     * @return Itself, with this parameter changed.
     */
    public BeakMotionMagicDistance withPosition(Measure<Distance> position) {
        this.Position = position;
        return this;
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param useFOC Whether or not to use FOC.
     * @return Itself, with this parameter changed.
     */
    public BeakMotionMagicDistance withUseFOC(boolean useFOC) {
        this.UseFOC = useFOC;
        return this;
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param type What output type to use: voltage, duty cycle, current
     * @return Itself, with this parameter changed.
     */
    public BeakMotionMagicDistance withOutputType(OutputType type) {
        this.Output = type;
        return this;
    }

}
