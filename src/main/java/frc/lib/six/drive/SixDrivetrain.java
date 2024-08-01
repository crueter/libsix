// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.six.drive;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.six.gyro.SixGyro;

/** Base drivetrain class. */
public class SixDrivetrain extends SubsystemBase {
    protected Pose2d m_pose;

    protected BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();
    protected double m_lastAccel = 0.;

    protected double m_jerk = 0.;

    protected DrivetrainConfiguration m_config;

    protected SixGyro m_gyro;
    protected AnalogGyroSim m_gyroSim;

    /**
     * Construct a new generic drivetrain.
     * 
     * @param config The drivetrain's configuration.
     */
    public SixDrivetrain(DrivetrainConfiguration config) {
        m_config = config;
        m_gyroSim = new AnalogGyroSim(0);
    }

    public void configMotors() {
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param x
     *                      Speed of the robot in the x direction (forward).
     * @param y
     *                      Speed of the robot in the y direction (sideways).
     * @param rot
     *                      Angular rate of the robot.
     * @param fieldRelative
     *                      Whether or not the x and y speeds are relative to
     *                      the field, for holonomic drivetrains.
     */
    public void drive(double x, double y, double rot, boolean fieldRelative) {
        x *= m_config.MaxSpeed;
        y *= m_config.MaxSpeed;
        rot *= m_config.MaxAngularVelocity;

        ChassisSpeeds speeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, getRotation2d())
                : new ChassisSpeeds(x, y, rot);

        drive(speeds);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param x
     *            Speed of the robot in the x direction (forward).
     * @param y
     *            Speed of the robot in the y direction (sideways).
     * @param rot
     *            Angular rate of the robot.
     */
    public void drive(double x, double y, double rot) {
        drive(x, y, rot, false);
    }

    /**
     * Method to drive the robot using chassis speeds
     * 
     * @param speeds
     *               The ChassisSpeeds to use.
     */
    public void drive(ChassisSpeeds speeds) {
    }

    /**
     * Get the robot's pose.
     * 
     * @return The pose reported from the odometry, measured in meters.
     */
    public Pose2d getPoseMeters() {
        return null;
    }

    /**
     * Get the robot's rotation.
     * 
     * @return A {@link Rotation2d} of the reported robot rotation from the
     *         odometry.
     */
    public Rotation2d getRotation2d() {
        return getPoseMeters().getRotation();
    }

    /**
     * Get the robot's heading.
     * 
     * @return The heading reported from the odometry, in degrees.
     */
    public double getHeading() {
        return getRotation2d().getDegrees();
    }

    /**
     * Reset odometry to specified pose.
     * 
     * @param pose
     *             Pose to reset odometry to.
     */
    public void resetOdometry(Pose2d pose) {
    }

    /**
     * Update the robot odometry.
     * 
     * @return Updated pose.
     */
    public Pose2d updateOdometry() {
        return m_pose;
    }

    /**
     * Add a vision measurement to the pose estimator's
     * Kalman filter.
     * 
     * @param estimatedPose The estimated pose of the robot from the vision system.
     * @param timestamp The timestamp of the received data.
     */
    public void addVisionMeasurement(Pose2d estimatedPose, double timestamp) {
    }

    /**
     * <p>
     * Get the angle to a target position on the field.
     * </p>
     * 
     * Positions are relative to the bottom-left corner of the field (for Rapid
     * React, the blue alliance HP station)
     * 
     * @param x
     *          The X position of the target.
     * @param y
     *          The Y position of the target.
     * @return A {@link Rotation2d} of the drivetrain's angle to the target
     *         position.
     */
    public Rotation2d getAngleToTargetPosition(Measure<Distance> x, Measure<Distance> y) {
        double xDelta = x.in(Meters) - getPoseMeters().getX();
        double yDelta = y.in(Meters) - getPoseMeters().getY();

        double radiansToTarget = Math.atan2(yDelta, xDelta);

        return new Rotation2d(radiansToTarget);
    }

    /**
     * Determine whether or not this drivetrain is holonomic.
     * 
     * @return Whether or not the drivetrain is holonomic (e.g. swerve)
     */
    public boolean isHolonomic() {
        return false;
    }

    /**
     * Get the jerk
     * 
     * @return todo
     */
    public double getJerk() {
        return m_jerk;
    }

    /**
     * Gets the gyro's reported angle.
     * 
     * @return A {@link Rotation2d} containing the reported angle of the gyro.
     */
    public Rotation2d getGyroRotation2d() {
        if (RobotBase.isSimulation()) {
            return Rotation2d.fromDegrees(m_gyroSim.getAngle());
        } else {
            return m_gyro.getYawRotation2d(true).getValue();
        }
    }

    /**
     * Get the gyro's reported heading.
     * 
     * @return The reported angle of the gyro in degrees.
     */
    public double getGyroHeading() {
        return getGyroRotation2d().getDegrees();
    }

    /**
     * Get the gyro's reported rate.
     * 
     * @return The reported rate of the gyro in degrees per second.
     */
    public double getGyroRate() {
        if (RobotBase.isSimulation()) {
            return m_gyroSim.getRate();
        } else {
            return m_gyro.getAngularVelocity().getValue().in(DegreesPerSecond);
        }
    }

    /**
     * Get the gyro's reported pitch.
     * 
     * @return The reported pitch of the gyro.
     */
    public Rotation2d getGyroPitchRotation2d() {
        return m_gyro.getPitchRotation2d(true).getValue();
    }

    /**
     * Get the gyro's reported roll.
     * 
     * @return The reported roll of the gyro.
     */
    public Rotation2d getGyroRollRotation2d() {
        return m_gyro.getRollRotation2d(true).getValue();
    }

    @Override
    public void periodic() {
        m_jerk = (m_accelerometer.getY() - m_lastAccel);
    }
}