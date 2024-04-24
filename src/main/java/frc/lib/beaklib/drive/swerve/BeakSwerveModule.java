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
import edu.wpi.first.math.util.Units;
import frc.lib.beaklib.encoder.BeakAbsoluteEncoder;
import frc.lib.beaklib.motor.BeakMotorController;

/** Base class for any non-differential swerve module. */
public class BeakSwerveModule {
    public SwerveModuleConfiguration Config;

    protected BeakMotorController m_driveMotor;
    protected BeakMotorController m_steerMotor;
    protected BeakAbsoluteEncoder m_steerEncoder;

    public enum DriveRequestType {
        VelocityFOC,
        Velocity,
        VoltageFOC,
        Voltage
    }

    public enum SteerRequestType {
        MotionMagic,
        MotionMagicFOC
    }

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
        m_steerMotor = turningMotor;
        m_steerEncoder = turningEncoder;

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
        // System.err.println(Config.DriveInverted);
        m_driveMotor.setVoltageCompensationSaturation(12.0);
        m_driveMotor.setSupplyCurrentLimit(Config.DriveConfig.DriveSupplyLimit);
        m_driveMotor.setStatorCurrentLimit(Config.DriveConfig.DriveStatorLimit);

        // Configure PID
        m_driveMotor.setPID(Config.DriveConfig.DrivePID);
    }

    public void configTurningMotor() {
        m_steerMotor.setEncoderGearRatio(Config.DriveConfig.SteerRatio);

        m_steerMotor.setBrake(true);
        m_steerMotor.setInverted(Config.SteerInverted);

        // Initialize the encoder's position--MUST BE DONE AFTER
        // CONFIGURING TURNING ENCODER!
        resetTurningMotor();

        // Generally, turning motor current draw isn't a problem.
        // This is done to prevent stalls from killing the motor.
        m_steerMotor.setSupplyCurrentLimit(Config.DriveConfig.SteerCurrentLimit);

        m_steerMotor.setMotionMagicCruiseVelocity(100.0 / Config.DriveConfig.SteerRatio);
        m_steerMotor.setMotionMagicAcceleration(1000.0 / Config.DriveConfig.SteerRatio);

        // m_steerMotor.setVoltageCompensationSaturation(0.);

        m_steerMotor.setPID(Config.DriveConfig.TurnPID);
    }

    public void configTurningEncoder() {
        m_steerEncoder.setAbsoluteOffset(Config.AngleOffset);

        // Prevent huge CAN spikes
        m_steerEncoder.setDataFramePeriod(101);
    }

    /* Bruh */
    public BeakMotorController getDriveMotor() {
        return m_driveMotor;
    }

    public BeakMotorController getSteerMotor() {
        return m_steerMotor;
    }

    public BeakAbsoluteEncoder getSteerEncoder() {
        return m_steerEncoder;
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
        m_steerMotor.setEncoderPositionMotorRotations(
                Math.toDegrees(getAbsoluteEncoderRadians()) / 360.0);
    }

    /**
     * Get the angle of the wheel.
     * 
     * @return Angle of the wheel in radians.
     */
    public double getAbsoluteEncoderRadians() {
        double angle = m_steerEncoder.getAbsoluteEncoderPosition(false).Value.getRadians();
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    public double getTurningEncoderRadians() {
        double angle = m_steerMotor.getAngle(false).Value.getRadians();

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
        m_steerMotor.setEncoderPositionNU(0);
    }

    /**
     * Applies the desired SwerveModuleState to this module.
     *
     * @param state            Speed and direction the module should target
     * @param driveRequestType The {@link DriveRequestType} to apply
     */
    public void apply(SwerveModuleState state, DriveRequestType driveRequestType) {
        apply(state, driveRequestType, SteerRequestType.MotionMagic);
    }

    /**
     * Applies the desired SwerveModuleState to this module.
     *
     * @param state            Speed and direction the module should target
     * @param driveRequestType The {@link DriveRequestType} to apply
     * @param steerRequestType The {@link SteerRequestType} to apply; defaults to {@link SteerRequestType#MotionMagic}
     */
    public void apply(SwerveModuleState state, DriveRequestType driveRequestType, SteerRequestType steerRequestType) {
        var optimized = SwerveModuleState.optimize(state, m_steerMotor.getAngle(true).Value);

        double angleToSetDeg = optimized.angle.getRotations();
        switch (steerRequestType) {
            // TODO: Implement FOC
            case MotionMagic:
            case MotionMagicFOC:
                m_steerMotor.setMotionMagicAngle(Rotation2d.fromDegrees(angleToSetDeg));
                break;
        }

        double velocityToSet = optimized.speedMetersPerSecond;

        /* From FRC 900's whitepaper, we add a cosine compensator to the applied drive velocity */
        /* To reduce the "skew" that occurs when changing direction */
        double steerMotorError = angleToSetDeg - m_steerMotor.getAngle(true).Value.getDegrees();

        /* If error is close to 0 rotations, we're already there, so apply full power */
        /* If the error is close to 0.25 rotations, then we're 90 degrees, so movement doesn't help us at all */
        double cosineScalar = Math.cos(Units.rotationsToRadians(steerMotorError));
        
        /* Make sure we don't invert our drive, even though we shouldn't ever target over 90 degrees anyway */
        if (cosineScalar < 0.0) {
            cosineScalar = 0.0;
        }
        velocityToSet *= cosineScalar;

        switch (driveRequestType) {
            case Voltage:
            case VoltageFOC:
                m_driveMotor.setVoltage(velocityToSet / Config.DriveConfig.MaxSpeed * 12.0);
                break;

            case Velocity:
            case VelocityFOC:
                m_driveMotor.setVelocity(MetersPerSecond.of(velocityToSet));
                break;
        }
    }
}
