// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.drive.swerve;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.beaklib.encoder.BeakAbsoluteEncoder;
import frc.lib.beaklib.motor.BeakMotorController;

/** Base class for any non-differential swerve module. */
public class BeakSwerveModule {
    public SwerveModuleConfiguration Config;

    protected BeakMotorController m_driveMotor;
    protected BeakMotorController m_turningMotor;
    protected BeakAbsoluteEncoder m_turningEncoder;

    /**
     * Construct a new Swerve Module.
     * 
     * @param config
     *               {@link SwerveModuleconfiguration} containing
     *               details of the module.
     */
    public BeakSwerveModule(SwerveModuleConfiguration config) {
        Config = config;
    }

    /**
     * Call this function in a subclass AFTER setting up motors and encoders
     */
    public void setup(
            BeakMotorController driveMotor,
            BeakMotorController turningMotor,
            BeakAbsoluteEncoder turningEncoder) {
        m_driveMotor = driveMotor;
        m_turningMotor = turningMotor;
        m_turningEncoder = turningEncoder;

        configTurningEncoder();
        configTurningMotor();
        configDriveMotor();
    }

    public void configDriveMotor() {
        m_driveMotor.setEncoderGearRatio(Config.DriveConfig.DriveRatio);
        m_driveMotor.setWheelDiameter(Inches.of(Config.DriveConfig.WheelDiameter));

        m_driveMotor.setBrake(true);
        m_driveMotor.setInverted(Config.DriveInverted);

        // Prevent the motors from drawing several hundred amps of current,
        // and allow them to run at the same speed even when voltage drops.
        m_driveMotor.setVoltageCompensationSaturation(12.0);
        m_driveMotor.setSupplyCurrentLimit(Config.DriveConfig.DriveSupplyLimit);
        m_driveMotor.setStatorCurrentLimit(Config.DriveConfig.DriveStatorLimit);

        // Configure PID
        m_driveMotor.setPID(Config.DriveConfig.DrivePID);
    }

    public void configTurningMotor() {
        m_turningMotor.setEncoderGearRatio(Config.DriveConfig.TurnRatio);

        m_turningMotor.setBrake(true);
        m_turningMotor.setInverted(Config.TurnInverted);

        // Initialize the encoder's position--MUST BE DONE AFTER
        // m_configURING TURNING ENCODER!
        resetTurningMotor();

        // Generally, turning motor current draw isn't a problem.
        // This is done to prevent stalls from killing the motor.
        m_turningMotor.setSupplyCurrentLimit(Config.DriveConfig.TurnCurrentLimit);

        m_turningMotor.setVoltageCompensationSaturation(0.);

        m_turningMotor.setPID(Config.DriveConfig.TurnPID);
    }

    public void configTurningEncoder() {
        m_turningEncoder.setAbsoluteOffset(Config.AngleOffset);

        // Prevent huge CAN spikes
        m_turningEncoder.setDataFramePeriod(101);
    }

    /* State Management */

    /**
     * Get the module's current state.
     * 
     * @return Current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                m_driveMotor.getSpeed().Value.in(MetersPerSecond),
                new Rotation2d(getAbsoluteEncoderRadians())); // FUTURE: Using Absolute reverses some wheels.
    }

    /**
     * Get the module's current position.
     * 
     * @return Current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                m_driveMotor.getDistance(true).Value.in(Meters),
                new Rotation2d(getTurningEncoderRadians()));
    }

    /** Encoders & Heading */

    /**
     * Set the turning motor's position to match the reported
     * angle from the CANCoder.
     */
    public void resetTurningMotor() {
        m_turningMotor.setEncoderPositionMotorRotations(
                Math.toDegrees(getAbsoluteEncoderRadians()) / 360.0);
    }

    /**
     * Get the angle of the wheel.
     * 
     * @return Angle of the wheel in radians.
     */
    public double getAbsoluteEncoderRadians() {
        double angle = m_turningEncoder.getAbsoluteEncoderPosition(false).Value.getRadians();
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    public double getTurningEncoderRadians() {
        double angle = m_turningMotor.getAngle(false).Value.getRadians();

        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    /**
     * Zero all encoders, in case things have gone bad
     */
    public void resetEncoders() {
        m_driveMotor.setEncoderPositionNU(0);
        m_turningMotor.setEncoderPositionNU(0);
    }

    // /**
    //  * Set the wheel's angle.
    //  * 
    //  * @param newAngle
    //  *                 Angle to turn the wheel to, in degrees.
    //  */
    // public void setAngle(double newAngle) {
    //     // Does some funky stuff to do the cool thing
    //     double currentSensorPosition = m_turningMotor.getPositionMotorRotations(false).Value * 360.0;
    //     double remainder = Math.IEEEremainder(currentSensorPosition, 360.0);
    //     double newAngleDemand = newAngle + currentSensorPosition - remainder;

    //     if (newAngleDemand - currentSensorPosition > 180.1) {
    //         newAngleDemand -= 360.0;
    //     } else if (newAngleDemand - currentSensorPosition < -180.1) {
    //         newAngleDemand += 360.0;
    //     }

    //     m_turningMotor.setPositionMotorRotations(newAngleDemand / 360.0);
    // }
}
