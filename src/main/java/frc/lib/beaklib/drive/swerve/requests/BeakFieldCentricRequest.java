// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.drive.swerve.requests;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.beaklib.drive.swerve.BeakSwerveModule;

/** Add your docs here. */
public class BeakFieldCentricRequest implements BeakSwerveRequest {
    /**
     * The velocity in the X direction, in m/s.
     * X is defined as forward according to WPILib convention,
     * so this determines how fast to travel forward.
     */
    public double VelocityX = 0;
    /**
     * The velocity in the Y direction, in m/s.
     * Y is defined as to the left according to WPILib convention,
     * so this determines how fast to travel to the left.
     */
    public double VelocityY = 0;
    /**
     * The angular rate to rotate at, in radians per second.
     * Angular rate is defined as counterclockwise positive,
     * so this determines how fast to turn counterclockwise.
     */
    public double RotationalRate = 0;
    /**
     * The allowable deadband of the request.
     */
    public double Deadband = 0;
    /**
     * The rotational deadband of the request.
     */
    public double RotationalDeadband = 0;
    /**
     * The center of rotation the robot should rotate around.
     * This is (0,0) by default, which will rotate around the center of the robot.
     */
    public Translation2d CenterOfRotation = new Translation2d();

    /**
     * The type of control request to use for the drive motor.
     */
    public BeakSwerveModule.DriveRequestType DriveRequestType = BeakSwerveModule.DriveRequestType.Voltage;

    /**
     * The type of control request to use for the steer motor.
     */
    public BeakSwerveModule.SteerRequestType SteerRequestType = BeakSwerveModule.SteerRequestType.MotionMagic;

    /**
     * The last applied state in case we don't have anything to drive.
     */
    protected SwerveModuleState[] m_lastAppliedState = null;

    // TODO: look forward based on acceleration
    public void apply(SwerveControlRequestParameters parameters, List<BeakSwerveModule> modulesToApply) {
        double toApplyX = VelocityX;
        double toApplyY = VelocityY;
        
        double toApplyOmega = RotationalRate;
        if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
            toApplyX = 0;
            toApplyY = 0;
        }
        if (Math.abs(toApplyOmega) < RotationalDeadband) {
            toApplyOmega = 0;
        }

        ChassisSpeeds speeds = ChassisSpeeds
                .discretize(ChassisSpeeds.fromFieldRelativeSpeeds(toApplyX, toApplyY, toApplyOmega,
                        parameters.currentPose.getRotation()), parameters.updatePeriod);

        var states = parameters.kinematics.toSwerveModuleStates(speeds, CenterOfRotation);

        for (int i = 0; i < modulesToApply.size(); ++i) {
            modulesToApply.get(i).apply(states[i], DriveRequestType, SteerRequestType);
        }
    }

    /**
     * Sets the velocity in the X direction, in m/s.
     * X is defined as forward according to WPILib convention,
     * so this determines how fast to travel forward.
     *
     * @param velocityX Velocity in the X direction, in m/s
     * @return this request
     */
    public BeakFieldCentricRequest withVelocityX(double velocityX) {
        this.VelocityX = velocityX;
        return this;
    }

    /**
     * Sets the velocity in the Y direction, in m/s.
     * Y is defined as to the left according to WPILib convention,
     * so this determines how fast to travel to the left.
     *
     * @param velocityY Velocity in the Y direction, in m/s
     * @return this request
     */
    public BeakFieldCentricRequest withVelocityY(double velocityY) {
        this.VelocityY = velocityY;
        return this;
    }

    /**
     * The angular rate to rotate at, in radians per second.
     * Angular rate is defined as counterclockwise positive,
     * so this determines how fast to turn counterclockwise.
     *
     * @param rotationalRate Angular rate to rotate at, in radians per second
     * @return this request
     */
    public BeakFieldCentricRequest withRotationalRate(double rotationalRate) {
        this.RotationalRate = rotationalRate;
        return this;
    }

    /**
     * Sets the allowable deadband of the request.
     *
     * @param deadband Allowable deadband of the request
     * @return this request
     */
    public BeakFieldCentricRequest withDeadband(double deadband) {
        this.Deadband = deadband;
        return this;
    }

    /**
     * Sets the rotational deadband of the request.
     *
     * @param rotationalDeadband Rotational deadband of the request
     * @return this request
     */
    public BeakFieldCentricRequest withRotationalDeadband(double rotationalDeadband) {
        this.RotationalDeadband = rotationalDeadband;
        return this;
    }

    /**
     * Sets the center of rotation of the request
     *
     * @param centerOfRotation The center of rotation the robot should rotate
     *                         around.
     * @return this request
     */
    public BeakFieldCentricRequest withCenterOfRotation(Translation2d centerOfRotation) {
        this.CenterOfRotation = centerOfRotation;
        return this;
    }

    /**
     * Sets the type of control request to use for the drive motor.
     *
     * @param driveRequestType The type of control request to use for the drive
     *                         motor
     * @return this request
     */
    public BeakFieldCentricRequest withDriveRequestType(BeakSwerveModule.DriveRequestType driveRequestType) {
        this.DriveRequestType = driveRequestType;
        return this;
    }

    /**
     * Sets the type of control request to use for the steer motor.
     *
     * @param steerRequestType The type of control request to use for the steer
     *                         motor
     * @return this request
     */
    public BeakFieldCentricRequest withSteerRequestType(BeakSwerveModule.SteerRequestType steerRequestType) {
        this.SteerRequestType = steerRequestType;
        return this;
    }
}
