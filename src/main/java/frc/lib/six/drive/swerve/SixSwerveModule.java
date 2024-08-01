// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.six.drive.swerve;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.six.encoder.SixAbsoluteEncoder;
import frc.lib.six.motor.SixMotorController;
import frc.lib.six.motor.DataSignal;
import frc.lib.six.motor.configs.SixCurrentLimitConfigs;
import frc.lib.six.motor.requests.SixVoltage;
import frc.lib.six.motor.requests.motionmagic.SixMotionMagicAngle;
import frc.lib.six.motor.requests.position.SixPositionAngle;
import frc.lib.six.motor.requests.velocity.SixVelocity;

/** Base class for any non-differential swerve module. */
public class SixSwerveModule {
    public SwerveModuleConfiguration Config;

    protected SixMotorController m_driveMotor;
    protected SixMotorController m_steerMotor;
    protected SixAbsoluteEncoder m_steerEncoder;

    private SixCurrentLimitConfigs m_driveCurrentLimits = new SixCurrentLimitConfigs();
    private SixCurrentLimitConfigs m_steerCurrentLimits = new SixCurrentLimitConfigs();

    public enum DriveRequestType {
        VelocityFOC,
        Velocity,
        VoltageFOC,
        Voltage
    }

    public enum SteerRequestType {
        MotionMagic,
        MotionMagicFOC,
        Position,
        PositionFOC
    }

    // TODO: implement motion magic fr
    protected SixMotionMagicAngle m_motionMagicAngle = new SixMotionMagicAngle();
    protected SixPositionAngle m_positionAngle = new SixPositionAngle();
    protected SixVelocity m_velocity = new SixVelocity();
    protected SixVoltage m_voltage = new SixVoltage();

    protected DataSignal<Rotation2d> m_steerMotorAngle;
    protected DataSignal<Rotation2d> m_absoluteAngle;
    protected DataSignal<Measure<Distance>> m_driveDistance;
    // protected DataSignal<Measure<Velocity<Distance>>> m_driveSpeed;
    protected DataSignal<Double> m_driveSpeed;

    /**
     * Construct a new Swerve Module.
     * 
     * @param config
     *               {@link SwerveModuleConfiguration} containing
     *               details of the module.
     */
    public SixSwerveModule(SwerveModuleConfiguration config) {
        Config = config;
    }

    /**
     * Call this function in a subclass AFTER setting up motors and encoders
     */
    public void setup(
            SixMotorController driveMotor,
            SixMotorController steerMotor,
            SixAbsoluteEncoder steerEncoder) {
        m_driveMotor = driveMotor;
        m_steerMotor = steerMotor;
        m_steerEncoder = steerEncoder;

        configSteerEncoder();
        configSteerMotor();
        configDriveMotor();
    }

    public void configDriveMotor() {
        m_driveMotor.setEncoderGearRatio(Config.DriveConfig.DriveRatio);
        m_driveMotor.setWheelDiameter(Inches.of(Config.DriveConfig.WheelDiameter));

        m_driveMotor.setBrake(true);
        m_driveMotor.setInverted(Config.DriveInverted);

        m_driveMotor.setNominalVoltage(12.0);

        // Prevent the motors from drawing several hundred amps of current,
        // and allow them to run at the same speed even when voltage drops.
        // System.err.println(Config.DriveInverted);
        m_driveMotor.applyConfig(m_driveCurrentLimits
                .withStatorCurrentLimit(Config.DriveConfig.DriveStatorLimit)
                .withSupplyCurrentLimit(Config.DriveConfig.DriveSupplyLimit));

        // Configure PID
        m_driveMotor.setPID(Config.DriveConfig.DrivePID);
        System.out.println(Config.DriveConfig.SteerPID.kP);

        m_driveSpeed = m_driveMotor.getVelocityNU();
        m_driveDistance = m_driveMotor.getDistance(true);
    }

    public void configSteerMotor() {
        m_steerMotor.setEncoderGearRatio(Config.DriveConfig.SteerRatio);

        m_steerMotor.setBrake(true);
        m_steerMotor.setInverted(Config.SteerInverted);

        // Initialize the encoder's position--MUST BE DONE AFTER
        // CONFIGURING TURNING ENCODER!
        resetSteerMotor();

        // Generally, turning motor current draw isn't a problem.
        // This is done to prevent stalls from killing the motor.
        m_steerMotor.applyConfig(m_steerCurrentLimits
                .withSupplyCurrentLimit(Config.DriveConfig.SteerCurrentLimit));

        m_steerMotor.setNominalVoltage(12.0);

        m_steerMotor.setPID(Config.DriveConfig.SteerPID);

        m_steerMotorAngle = m_steerMotor.getAngle(true);
    }

    public void configSteerEncoder() {
        m_steerEncoder.setAbsoluteOffset(Config.AngleOffset);

        // Prevent huge CAN spikes
        m_steerEncoder.setDataFramePeriod(21);

        m_absoluteAngle = m_steerEncoder.getAbsoluteEncoderPosition(true);
    }

    /* Bruh */
    public SixMotorController getDriveMotor() {
        return m_driveMotor;
    }

    public SixMotorController getSteerMotor() {
        return m_steerMotor;
    }

    public SixAbsoluteEncoder getSteerEncoder() {
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
                m_driveSpeed.getValue(),//.in(MetersPerSecond),
                new Rotation2d(getAbsoluteEncoderRadians())); // FUTURE: Using Absolute reverses some wheels.
    }

    /**
     * Get the module's current position.
     * 
     * @return Current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                m_driveDistance.getValue().in(Meters),
                new Rotation2d(getSteerEncoderRadians()));
    }

    /** Encoders & Heading */

    /**
     * Set the turning motor's position to match the reported
     * angle from the CANCoder.
     */
    public void resetSteerMotor() {
        m_steerMotor.setEncoderPositionMotorRotations(
                Math.toDegrees(getAbsoluteEncoderRadians()) / 360.0);
    }

    /**
     * Get the angle of the wheel.
     * 
     * @return Angle of the wheel in radians.
     */
    public double getAbsoluteEncoderRadians() {
        double angle = m_absoluteAngle.getValue().getRadians();
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    public double getSteerEncoderRadians() {
        double angle = m_steerMotorAngle.getValue().getRadians();

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
        apply(state, driveRequestType, SteerRequestType.Position);
    }

    /**
     * Applies the desired SwerveModuleState to this module.
     *
     * @param state            Speed and direction the module should target
     * @param driveRequestType The {@link DriveRequestType} to apply
     * @param steerRequestType The {@link SteerRequestType} to apply; defaults to
     *                         {@link SteerRequestType#MotionMagic}
     */
    public void apply(SwerveModuleState state, DriveRequestType driveRequestType, SteerRequestType steerRequestType) {
        var optimized = SwerveModuleState.optimize(state, m_steerMotorAngle.getValue());

        double angleToSetDeg = optimized.angle.getDegrees();

        // m_steerMotor.setAngle(Rotation2d.fromDegrees(angleToSetDeg));
        // m_steerMotor.setControl(m_positionAngle.withAngle(Rotation2d.fromDegrees(angleToSetDeg)));
        switch (steerRequestType) {
            case MotionMagic:
                m_steerMotor.setControl(m_motionMagicAngle.withAngle(Rotation2d.fromDegrees(angleToSetDeg)));
                break;
            case MotionMagicFOC:
                m_steerMotor.setControl(
                        m_motionMagicAngle.withAngle(Rotation2d.fromDegrees(angleToSetDeg)).withUseFOC(true));
                break;
            case Position:
                SmartDashboard.putNumber("Position " + Config.ModuleLocation.getAngle(), m_steerMotor.getPositionNU(false).getValue());

                m_steerMotor.setControl(m_positionAngle.withAngle(Rotation2d.fromDegrees(angleToSetDeg)));
                break;
            case PositionFOC:
                m_steerMotor.setControl(
                        m_positionAngle.withAngle(Rotation2d.fromDegrees(angleToSetDeg)).withUseFOC(true));
                break;
        }

        double velocityToSet = optimized.speedMetersPerSecond;

        /*
         * From FRC 900's whitepaper, we add a cosine compensator to the applied drive
         * velocity
         */
        /* To reduce the "skew" that occurs when changing direction */
        double steerMotorError = angleToSetDeg - m_steerMotorAngle.getValue().getDegrees();

        /* If error is close to 0 rotations, we're already there, so apply full power */
        /*
         * If the error is close to 0.25 rotations, then we're 90 degrees, so movement
         * doesn't help us at all
         */
        double cosineScalar = Math.cos(Units.degreesToRadians(steerMotorError));

        /*
         * Make sure we don't invert our drive, even though we shouldn't ever target
         * over 90 degrees anyway
         */
        if (cosineScalar < 0.0) {
            cosineScalar = 0.0;
        }
        velocityToSet *= cosineScalar;

        double volts = velocityToSet / Config.DriveConfig.MaxSpeed * 12.0;

        switch (driveRequestType) {
            case Voltage:
                SmartDashboard.putNumber("Volts", volts);
                SmartDashboard.putNumber("volts " + Config.ModuleLocation.getAngle(), volts);
                m_driveMotor.setControl(m_voltage.withVoltage(volts));
                // m_driveMotor.setVoltage(volts);
                // m_driveMotor.set(volts / 12.0);
                break;
            case VoltageFOC:
                // SmartDashboard.putNumber("Volts ", volts);
                // m_driveMotor.setControl(m_voltage.withVoltage(volts));
                // m_driveMotor.setVoltage(volts);

                m_driveMotor.setControl(m_voltage.withVoltage(volts).withUseFOC(true));
                break;
            case Velocity:
                m_driveMotor.setControl(m_velocity.withVelocity(MetersPerSecond.of(velocityToSet)));
                break;
            case VelocityFOC:
                m_driveMotor.setControl(m_velocity.withVelocity(MetersPerSecond.of(velocityToSet)).withUseFOC(true));
                break;
        }
    }
}
