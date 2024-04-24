// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.motor.requests.position;

import static edu.wpi.first.units.Units.Feet;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import frc.lib.beaklib.motor.BeakMotorController;
import frc.lib.beaklib.motor.requests.BeakControlRequest;

/** Drive to a specified distance. */
public class BeakPositionDistance extends BeakControlRequest {
    /**
     * Position setpoint.
     */
    public Measure<Distance> Position = Feet.zero();

    /**
     * Whether or not to use FOC commutation if supported.
     */
    public boolean UseFOC = false;

    public BeakPositionDistance() {
    }

    public void apply(BeakMotorController controller) {
        controller.useFOC(UseFOC);
        controller.setPosition(Position);
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param position The new position to use.
     * @return Itself, with this parameter changed.
     */
    public BeakPositionDistance withPosition(Measure<Distance> position) {
        this.Position = position;
        return this;
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param useFOC Whether or not to use FOC.
     * @return Itself, with this parameter changed.
     */
    public BeakPositionDistance withUseFOC(boolean useFOC) {
        this.UseFOC = useFOC;
        return this;
    }
}
