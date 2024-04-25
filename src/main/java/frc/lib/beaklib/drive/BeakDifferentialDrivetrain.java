// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.drive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;

import frc.lib.beaklib.drive.swerve.DrivetrainConfiguration;
import frc.lib.beaklib.gyro.BeakGyro;
import frc.lib.beaklib.motor.BeakMotorControllerGroup;
import frc.lib.beaklib.motor.configs.BeakCurrentLimitConfigs;

/** Base class for all differential (tank, kitbot, WCD) drivetrains. */
public class BeakDifferentialDrivetrain extends BeakDrivetrain {
    protected BeakMotorControllerGroup m_leftControllers;
    protected BeakMotorControllerGroup m_rightControllers;

    protected DifferentialDrivePoseEstimator m_odom;
    protected DifferentialDriveKinematics m_kinematics;

    private BeakCurrentLimitConfigs m_currentLimits = new BeakCurrentLimitConfigs();

    /**
     * Create a new Differential Drivetrain.
     * 
     * @param config
     *               {@link DrivetrainConfiguration} for this drivetrain
     */
    public BeakDifferentialDrivetrain(DrivetrainConfiguration config) {
        super(config);

        m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(m_config.TrackWidth));
        m_odom = new DifferentialDrivePoseEstimator(m_kinematics, getGyroRotation2d(), 0., 0., new Pose2d());
    }

    /**
     * Setup motor and gyro configurations.
     * 
     * @param leftMotorControllers  Motor Controllers that control the left side of
     *                              the drivetrain.
     * @param rightMotorControllers Motor Controllers that control the right side of
     *                              the drivetrain.
     * @param gyro                  The gyroscope in use for this drivetrain.
     */
    public void setup(
            BeakMotorControllerGroup leftMotorControllers,
            BeakMotorControllerGroup rightMotorControllers,
            BeakGyro gyro) {
        m_gyro = gyro;

        m_currentLimits.StatorCurrentLimit = m_config.DriveStatorLimit;
        m_currentLimits.SupplyCurrentLimit = m_config.DriveSupplyLimit;

        m_leftControllers = leftMotorControllers;
        m_rightControllers = rightMotorControllers;

        // Real-World Values
        m_leftControllers.setEncoderGearRatio(m_config.DriveRatio);
        m_leftControllers.setWheelDiameter(Inches.of(m_config.WheelDiameter));

        m_rightControllers.setEncoderGearRatio(m_config.DriveRatio);
        m_rightControllers.setWheelDiameter(Inches.of(m_config.WheelDiameter));

        // Current
        m_leftControllers.applyConfig(m_currentLimits);
        m_rightControllers.applyConfig(m_currentLimits);

        // PID
        m_leftControllers.setPID(m_config.DrivePID);
        m_rightControllers.setPID(m_config.DrivePID);
    }

    /* Differential-specific methods */

    /**
     * Open-loop drive with voltage input.
     * 
     * @param leftVolts
     *                   Volts to send to the left side.
     * @param rightVolts
     *                   Volts to send to the right side.
     */
    public void driveVolts(double leftVolts, double rightVolts) {
        m_leftControllers.setVoltage(leftVolts);
        m_rightControllers.setVoltage(rightVolts);
    }

    /**
     * Closed-loop drive with direct velocity input.
     * 
     * @param leftMetersPerSecond
     *                             Left-side velocity.
     * @param rightMetersPerSecond
     *                             Right-side velocity.
     */
    public void drive(double leftMetersPerSecond, double rightMetersPerSecond) {
        m_leftControllers.setVelocity(MetersPerSecond.of(leftMetersPerSecond));
        m_rightControllers.setVelocity(MetersPerSecond.of(rightMetersPerSecond));
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                m_leftControllers.getSpeed().Value.in(MetersPerSecond),
                m_rightControllers.getSpeed().Value.in(MetersPerSecond));
    }

    @Override
    public void drive(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);

        drive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    }

    @Override
    public Pose2d updateOdometry() {
        m_odom.updateWithTime(
                RobotController.getFPGATime() / 1000000.,
                getGyroRotation2d(),
                m_leftControllers.getDistance(true).Value.in(Meters),
                m_rightControllers.getDistance(true).Value.in(Meters));

        return getPoseMeters();
    }

    @Override
    public void addVisionMeasurement(Pose2d estimatedPose, double timestamp) {
        Transform2d poseError = estimatedPose.minus(m_odom.getEstimatedPosition());

        if (!estimatedPose.equals(new Pose2d()) && !estimatedPose.equals(getPoseMeters()) &&
                Math.abs(poseError.getX()) < 0.5 &&
                Math.abs(poseError.getY()) < 0.5) {
            m_odom.addVisionMeasurement(estimatedPose, timestamp);
        }
    }

    @Override
    public Pose2d getPoseMeters() {
        return m_odom.getEstimatedPosition();
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        if (!pose.equals(new Pose2d()))
            m_odom.resetPosition(getGyroRotation2d(),
                    m_leftControllers.getDistance(true).Value.in(Meters),
                    m_rightControllers.getDistance(true).Value.in(Meters),
                    pose);
    }

    @Override
    public boolean isHolonomic() {
        return false;
    }
}
