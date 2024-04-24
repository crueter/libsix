// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.motor.requests.position;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.beaklib.motor.BeakMotorController;
import frc.lib.beaklib.motor.requests.BeakControlRequest;

/** Drive to a specified angle. */
public class BeakPositionAngle extends BeakControlRequest {
    /**
     * Angle setpoint.
     */
    public Rotation2d Angle = new Rotation2d();

    /**
     * Whether or not to use FOC commutation if supported.
     */
    public boolean UseFOC = false;

    public BeakPositionAngle() {
    }

    public void apply(BeakMotorController controller) {
        controller.useFOC(UseFOC);
        controller.setAngle(Angle);
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param angle The new angle to use.
     * @return Itself, with this parameter changed.
     */
    public BeakPositionAngle withAngle(Rotation2d angle) {
        this.Angle = angle;
        return this;
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param useFOC Whether or not to use FOC.
     * @return Itself, with this parameter changed.
     */
    public BeakPositionAngle withUseFOC(boolean useFOC) {
        this.UseFOC = useFOC;
        return this;
    }
}
