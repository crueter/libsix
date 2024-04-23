// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.drive;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import frc.lib.beaklib.drive.swerve.DrivetrainConfiguration;
import frc.lib.beaklib.subsystem.BeakGyroSubsystem;

/** Base drivetrain class. */
public class BeakDrivetrain extends BeakGyroSubsystem {
    protected Pose2d m_pose;

    protected BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();
    protected double m_lastAccel = 0.;

    protected double m_jerk = 0.;

    protected DrivetrainConfiguration m_config;

    /**
     * Construct a new generic drivetrain.
     * 
     * @param config The drivetrain's configuration.
     */
    public BeakDrivetrain(DrivetrainConfiguration config) {
        m_config = config;
    }

    public void configMotors() {
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
     * @param fieldRelative
     *            Whether or not the x and y speeds are relative to
     *            the field, for holonomic drivetrains.
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
     *            The ChassisSpeeds to use.
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
     *            Pose to reset odometry to.
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
     * Add a vision measurement directly from PhotonVision to the pose estimator's Kalman filter.
     * 
     * @param estimatedPose The {@link EstimatedRobotPose} from PhotonVision.
     */
    public void addVisionMeasurement(Pose2d estimatedPose, double timestamp) {
    }

    /**
     * Get the angle to a target position on the field.
     * </p>
     * 
     * Positions are relative to the bottom-left corner of the field (for Rapid
     * React, the blue alliance HP station)
     * 
     * @param x
     *            The X position of the target.
     * @param y
     *            The Y position of the target.
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
     * @return todo
     */
    public double getJerk() {
        return m_jerk;
    }

    @Override
    public void periodic() {
        m_jerk = (m_accelerometer.getY() - m_lastAccel);
    }
}