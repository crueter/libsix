// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.six.motor.requests.position;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.six.motor.SixMotorController;
import frc.lib.six.motor.requests.SixControlRequest;

/** Drive to a specified angle. */
public class SixPositionAngle extends SixControlRequest {
    /**
     * Angle setpoint.
     */
    public Rotation2d Angle = new Rotation2d();

    /**
     * Whether or not to use FOC commutation if supported.
     */
    public boolean UseFOC = false;

    /**
     * Output value (current, voltage, duty cycle)
     */
    public OutputType Output = OutputType.Voltage;

    public SixPositionAngle() {
    }

    public void apply(SixMotorController controller) {
        controller.setNextOutputType(Output);
        controller.useFOC(UseFOC);
        controller.setAngle(Angle);
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param angle The new angle to use.
     * @return Itself, with this parameter changed.
     */
    public SixPositionAngle withAngle(Rotation2d angle) {
        this.Angle = angle;
        return this;
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param useFOC Whether or not to use FOC.
     * @return Itself, with this parameter changed.
     */
    public SixPositionAngle withUseFOC(boolean useFOC) {
        this.UseFOC = useFOC;
        return this;
    }

    /**
     * Method-chaining API for this request.
     * 
     * @param type What output type to use: voltage, duty cycle, current
     * @return Itself, with this parameter changed.
     */
    public SixPositionAngle withOutputType(OutputType type) {
        this.Output = type;
        return this;
    }

}
