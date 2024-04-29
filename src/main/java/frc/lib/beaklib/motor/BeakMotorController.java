// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.motor;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.lib.beaklib.motor.configs.BeakClosedLoopConfigs;
import frc.lib.beaklib.motor.configs.BeakCurrentConfigs;
import frc.lib.beaklib.motor.configs.BeakCurrentLimitConfigs;
import frc.lib.beaklib.motor.configs.BeakDutyCycleConfigs;
import frc.lib.beaklib.motor.configs.BeakHardwareLimitSwitchConfigs;
import frc.lib.beaklib.motor.configs.BeakMotionProfileConfigs;
import frc.lib.beaklib.motor.configs.BeakSoftLimitConfigs;
import frc.lib.beaklib.motor.configs.BeakVoltageConfigs;
import frc.lib.beaklib.motor.requests.BeakControlRequest;
import frc.lib.beaklib.motor.requests.BeakControlRequest.OutputType;
import frc.lib.beaklib.pid.BeakPIDConstants;

/** Common interface for all motor controllers. */
public interface BeakMotorController extends MotorController {
    /**
     * Set the motor to be on brake or coast mode.
     * 
     * @param brake
     *              True = brake, False = coast
     */
    public void setBrake(boolean brake);

    /**
     * Run the motor in current mode.
     *
     * @param amps The current, in amps, to run at.
     */
    public void setCurrent(double amps);

    /**
     * Run the motor in velocity mode.
     * </p>
     * 
     * To run in native units, use {@link setVelocityNU}.
     * 
     * @param velocity
     *                 Velocity to run.
     * 
     */
    default void setVelocity(Measure<Velocity<Distance>> velocity) {
        setVelocityNU(
                (velocity.in(MetersPerSecond) / (getWheelDiameter().in(Meters) * Math.PI) * 60.) // rpm
                        * getEncoderGearRatio() * getVelocityConversionConstant());
    }

    /**
     * Run the motor in velocity mode, based on an angular velocity target.
     * </p>
     * 
     * To run in native units, use {@link setVelocityNU}.
     * 
     * @param velocity Angular velocity to run.
     */
    default void setAngularVelocity(Measure<Velocity<Angle>> velocity) {
        setVelocityNU(velocity.in(RPM) * getVelocityConversionConstant() * getEncoderGearRatio());
    }

    /**
     * Run the motor in velocity mode, in NU.
     * </p>
     * NU/100ms for Talons, RPM for SparkMAX.
     * 
     * @param nu
     *           NU to run.
     */
    public void setVelocityNU(double nu);

    /**
     * Run the motor in position mode.
     * </p>
     * 
     * To run in native units, use {@link setPositionNU}.
     * 
     * @param distance
     *                 Distance to run.
     */
    default void setPosition(Measure<Distance> distance) {
        setPositionNU(
                (distance.in(Meters) * getPositionConversionConstant() * getEncoderGearRatio()) //
                        / (getWheelDiameter().in(Meters) * Math.PI));
    }

    /**
     * Run the motor to a specified angle.
     * </p>
     * 
     * To run in native units, use {@link setPositionNU}.
     * 
     * @param angle
     *              Angle to run to.
     */
    default void setAngle(Rotation2d angle) {
        setPositionNU(angle.getRotations() * getPositionConversionConstant() * getEncoderGearRatio());
    }

    /**
     * Run the motor in position mode, in NU.
     * </p>
     * 2048 NU per rotation for TalonFX, 4096 for TalonSRX, and usually 1 for
     * SparkMAX.
     * 
     * @param nu
     *           NU to run.
     */
    public void setPositionNU(double nu);

    /**
     * Sets the encoder's position.
     * </p>
     * 
     * To set in native units, use {@link setEncoderPositionNU}.
     * 
     * @param distance
     *                 Distance to set the encoder to.
     */
    default void setEncoderPosition(Measure<Distance> distance) {
        setEncoderPositionNU(
                (distance.in(Meters) * getPositionConversionConstant() * getEncoderGearRatio()) //
                        / (getWheelDiameter().in(Meters) * Math.PI));
    }

    /**
     * Sets the encoder's position, in motor rotations.
     * </p>
     * 
     * To set in native units, use {@link setEncoderPositionNU}.
     * 
     * @param rotations
     *                  Rotations to set the encoder to.
     */
    default void setEncoderPositionMotorRotations(double rotations) {
        setEncoderPositionNU(rotations * getPositionConversionConstant() * getEncoderGearRatio());
    }

    /**
     * Sets the encoder's position, in NU.
     * </p>
     * 2048 NU per rotation for TalonFX, 4096 for TalonSRX, and usually 1 for
     * SparkMAX.
     * 
     * @param nu
     *           NU to set the encoder to.
     */
    public void setEncoderPositionNU(double nu);

    /**
     * Resets the encoder position to 0.
     */
    default void resetEncoder() {
        setEncoderPositionNU(0.);
    }

    /**
     * Run the motor in motion magic mode.
     * </p>
     * 
     * To run in native units, use {@link setMotionProfileNU}.
     * 
     * @param distance
     *                 Distance to run.
     */
    default void setMotionProfile(Measure<Distance> distance) {
        setMotionProfileNU(
                (distance.in(Meters) * getPositionConversionConstant() * getEncoderGearRatio()) //
                        / (getWheelDiameter().in(Meters) * Math.PI));
    }

    /**
     * Runs the motor to a specified angle in motion magic mode.
     * </p>
     * 
     * To run in native units, use {@link setMotionProfileNU}.
     * 
     * @param angle
     *              Angle to run to.
     */
    default void setMotionProfileAngle(Rotation2d angle) {
        setMotionProfileNU(angle.getRotations() * getPositionConversionConstant() * getEncoderGearRatio());
    }

    /**
     * Runs the motor in motion magic mode, in NU.
     * </p>
     * 2048 NU per rotation for TalonFX, 4096 for TalonSRX, and usually 1 for
     * SparkMAX.S
     * 
     * @param nu
     *           NU to run.
     */
    public void setMotionProfileNU(double nu);

    /**
     * Set the arbitrary feedforward to pass to the next PID command.
     * 
     * @param arbFeedforward
     *                       The feedforward, in volts.
     */
    public void setNextArbFeedforward(double arbFeedforward);

    /**
     * Set the slot to use with PID.
     * 
     * @param slot
     *             The next slot to use with PID. This applies to both setting
     *             constants and using them.
     */
    public void setSlot(int slot);

    /**
     * Set the next output type to use.
     * 
     * @param outputType Voltage, duty cycle, or current
     */
    public void setNextOutputType(OutputType outputType);

    /**
     * Set the nominal voltage to use for voltage compensation, if available.
     * @param volts Nominal supply voltage.
     */
    public void setNominalVoltage(double volts);

    /**
     * Enable or disable FOC control.
     * 
     * @param useFoc Whether or not to use FOC (if supported by the motor)
     */
    public void useFOC(boolean useFoc);

    /**
     * Run the motor using the specified request.
     * 
     * @param request The arequest to apply.
     */
    default void setControl(BeakControlRequest request) {
        request.apply(this);
    }

    /**
     * Get the motor velocity.
     * 
     * @return Velocity combined with the timestamp of the received data.
     */
    default DataSignal<Measure<Velocity<Distance>>> getSpeed() {
        DataSignal<Double> velocity = getVelocityNU();

        return new DataSignal<Measure<Velocity<Distance>>>(
                () -> MetersPerSecond.of(velocity.getValue() * (getWheelDiameter().in(Meters) * Math.PI)
                        / getVelocityConversionConstant() / getEncoderGearRatio() / 60.),
                velocity::getTimestamp,
                velocity::refresh,
                velocity::setUpdateFrequency);
    }

    /**
     * Get the motor angular velocity.
     * 
     * @return Velocity combined with the timestamp of the received data.
     */
    default DataSignal<Measure<Velocity<Angle>>> getAngularVelocity() {
        DataSignal<Double> velocity = getVelocityNU();

        return new DataSignal<Measure<Velocity<Angle>>>(
                () -> RPM.of(velocity.getValue() / getVelocityConversionConstant() / getEncoderGearRatio()),
                velocity::getTimestamp,
                velocity::refresh,
                velocity::setUpdateFrequency);
    }

    /**
     * Get the motor velocity, in NU.
     * </p>
     * NU/100ms for Talons, RPM for SparkMAX.
     * 
     * @return Velocity in NU combined with the timestamp of the received data.
     */
    public DataSignal<Double> getVelocityNU();

    /**
     * Get the motor distance.
     * 
     * @param latencyCompensated
     *                           Whether or not to attempt latency compensation.
     * 
     * @return Distance combined with the timestamp of the received data.
     */
    default DataSignal<Measure<Distance>> getDistance(boolean latencyCompensated) {
        DataSignal<Double> position = getPositionNU(latencyCompensated);

        return new DataSignal<Measure<Distance>>(
                () -> Meters.of(position.getValue() * (getWheelDiameter().in(Meters) * Math.PI)
                        / getPositionConversionConstant() / getEncoderGearRatio()),
                position::getTimestamp,
                position::refresh,
                position::setUpdateFrequency);
    }

    /**
     * Get the motor position, in motor rotations.
     * 
     * @param latencyCompensated
     *                           Whether or not to attempt latency compensation.
     * 
     * @return Position in motor rotations.
     */
    default DataSignal<Rotation2d> getAngle(boolean latencyCompensated) {
        DataSignal<Double> position = getPositionNU(latencyCompensated);

        return new DataSignal<Rotation2d>(
                () -> Rotation2d
                        .fromRotations(position.getValue() / getPositionConversionConstant() / getEncoderGearRatio()),
                position::getTimestamp,
                position::refresh,
                position::setUpdateFrequency);
    }

    /**
     * Get the motor position, in NU.
     * 2048 NU per rotation for TalonFX, 4096 for TalonSRX, and usually 1 for
     * SparkMAX.
     * 
     * @param latencyCompensated
     *                           Whether or not to attempt latency compensation.
     * 
     * @return Position in NU combined with the timestamp of the received data.
     */
    public DataSignal<Double> getPositionNU(boolean latencyCompensated);

    /**
     * Stop the motor.
     */
    default void stop() {
        set(0.);
    }

    @Override
    default void disable() {
        stop();
    }

    @Override
    default void stopMotor() {
        stop();
    }

    /**
     * Get the voltage currently being run to the motor controller, with the
     * timestamp of the received data.
     */
    public DataSignal<Double> getSuppliedVoltage();

    /**
     * Get the current applied voltage to the motor controller.
     * 
     * @return Applied voltage.
     */
    default DataSignal<Double> getOutputVoltage() {
        DataSignal<Double> voltage = getSuppliedVoltage();

        return new DataSignal<Double>(
                () -> voltage.getValue() * get(),
                voltage::getTimestamp,
                voltage::refresh,
                voltage::setUpdateFrequency);
    }

    /**
     * Set PIDF gains.
     * 
     * @param constants
     *                  PIDF Constants.
     */
    public void setPID(BeakPIDConstants constants);

    /**
     * Get PIDF gains.
     */
    public BeakPIDConstants getPID();

    /* LIMIT SWITCH */
    // TODO: DOcs
    public boolean getForwardLimitSwitch();

    public boolean getReverseLimitSwitch();

    /* CONFIGS */

    public void applyConfig(BeakClosedLoopConfigs config);

    public void applyConfig(BeakCurrentLimitConfigs config);

    public void applyConfig(BeakDutyCycleConfigs config);

    public void applyConfig(BeakHardwareLimitSwitchConfigs config);

    public void applyConfig(BeakMotionProfileConfigs config);

    public void applyConfig(BeakVoltageConfigs config);

    public void applyConfig(BeakSoftLimitConfigs config);

    public void applyConfig(BeakCurrentConfigs config);

    /* CONVERSION API */

    /**
     * Set the velocity conversion constant for this motor.
     * </p>
     * 
     * The velocity conversion constant is a factor that, when dividing native
     * velocity units by the constant, outputs rotations per minute.
     * </p>
     * 
     * Default values:
     * <ul>
     * <li>v6 Talon FX, Spark MAX: 1 (NU are RPM)</li>
     * <li>v5 Talon FX: 600 / 2048 (NU/100ms -> RPM).</li>
     * <li>Talon SRX: 600 / 4096 (NU/100ms -> RPM)</li>
     * </ul>
     * 
     * @param constant
     *                 Conversion constant. Units: <code>NU/rev/min</code>
     */
    public void setVelocityConversionConstant(double constant);

    /**
     * Get the velocity conversion constant for this motor.
     * </p>
     * 
     * This is used by the RPM and m/s getter/setter methods. Divide the native
     * velocity units by this constant to output rotations per minute.
     * 
     * @return Conversion constant. Units: <code>NU/rev/min</code>
     */
    public double getVelocityConversionConstant();

    /**
     * Set the position conversion constant for this motor.
     * </p>
     * 
     * The position conversion constant is a factor that, when dividing native
     * position units by the constant, outputs rotations.
     * </p>
     * 
     * Default values:
     * <ul>
     * <li>v6 Talon FX, Spark MAX: 1 (NU are rotations)</li>
     * <li>v5 Talon FX: 2048 (NU -> rotations).</li>
     * <li>Talon SRX: 4096 (NU -> rotations)</li>
     * </ul>
     * 
     * @param constant
     *                 Conversion constant. Units: <code>NU/rev</code>
     */
    public void setPositionConversionConstant(double constant);

    /**
     * Get the position conversion constant for this motor.
     * </p>
     * 
     * This is used by the rotation and distance getter/setter methods. Divide the
     * native
     * position units by this constant to output rotations.
     * 
     * @return Conversion constant. Units: <code>NU/rev</code>
     */
    public double getPositionConversionConstant();

    /**
     * Set the gear ratio between the encoder and output shaft.
     * </p>
     * 
     * This number represents the number of rotations of the motor shaft per
     * rotation of the output shaft. Therefore, if a motor has a 16:1 gearbox
     * attached, this value should be 16.
     * </p>
     * 
     * For motors with integrated encoders, this will generally be greater than 1 if
     * the motor has a gearbox. However, if a non-integrated encoder is mounted
     * after the gearbox, this will be 1.
     * 
     * @param ratio
     *              Gear ratio. Units: coefficient
     */
    public void setEncoderGearRatio(double ratio);

    /**
     * Get the gear ratio between the encoder and output shaft.
     * </p>
     * 
     * This number represents the number of rotations of the motor shaft per
     * rotation of the output shaft. Therefore, if a motor has a 16:1 gearbox
     * attached, this value should be 16.
     * </p>
     * 
     * Divide the motor rotations or RPM by this number to get the actual rotations
     * or RPM of the final output shaft. Multiply rotations of the output shaft by
     * this number to get the number of motor rotations.
     * 
     * @return Gear ratio. Units: coefficient
     */
    public double getEncoderGearRatio();

    /**
     * Set the diameter of the wheel driven by this motor.
     * </p>
     * 
     * If the motor does not drive a traditional wheel but instead operates a linear
     * actuation mechanism, set this to the diameter of whatever circular object it
     * is rotating.
     * 
     * @param diameter
     *                 Diameter of the wheel. Units: distance
     */
    public void setWheelDiameter(Measure<Distance> diameter);

    /**
     * Get the diameter of the wheel driven by this motor.
     * </p>
     * 
     * Multiply the number of motor rotations or RPM by this number to get the
     * distance travelled by this motor, or the linear speed of the wheel attached
     * to it. Divide the speed or distance by this number to get output shaft
     * rotations.
     * </p>
     * 
     * Note that multiplying RPM by this will net meters per minute, so to get
     * meters per second, you need to divide by 60.
     * 
     * @return Diameter of the wheel. Units: distance
     */
    public Measure<Distance> getWheelDiameter();
}
