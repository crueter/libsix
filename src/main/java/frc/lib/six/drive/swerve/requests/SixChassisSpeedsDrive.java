// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.six.drive.swerve.requests;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.six.drive.swerve.SixSwerveModule;

/** Drive a swerve chassis based upon Chassis Speeds. */
public class SixChassisSpeedsDrive implements SixSwerveRequest {
    /**
     * The chassis speeds to apply to the drivetrain.
     */
    public ChassisSpeeds Speeds = new ChassisSpeeds();
    /**
     * The center of rotation to rotate around.
     */
    public Translation2d CenterOfRotation = new Translation2d(0, 0);
    /**
     * The type of control request to use for the drive motor.
     */
    public SixSwerveModule.DriveRequestType DriveRequestType = SixSwerveModule.DriveRequestType.Voltage;
    /**
     * The type of control request to use for the steer motor.
     */
    public SixSwerveModule.SteerRequestType SteerRequestType = SixSwerveModule.SteerRequestType.Position;

    public void apply(SwerveControlRequestParameters parameters, List<SixSwerveModule> modulesToApply) {
        var states = parameters.kinematics.toSwerveModuleStates(Speeds, CenterOfRotation);
        
        for (int i = 0; i < modulesToApply.size(); ++i) {
            modulesToApply.get(i).apply(states[i], DriveRequestType, SteerRequestType);
        }
    }

    /**
     * Sets the chassis speeds to apply to the drivetrain.
     *
     * @param speeds Chassis speeds to apply to the drivetrain
     * @return this request
     */
    public SixChassisSpeedsDrive withSpeeds(ChassisSpeeds speeds) {
        this.Speeds = speeds;
        return this;
    }

    /**
     * Sets the center of rotation to rotate around.
     *
     * @param centerOfRotation Center of rotation to rotate around
     * @return this request
     */
    public SixChassisSpeedsDrive withCenterOfRotation(Translation2d centerOfRotation) {
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
    public SixChassisSpeedsDrive withDriveRequestType(SixSwerveModule.DriveRequestType driveRequestType) {
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
    public SixChassisSpeedsDrive withSteerRequestType(SixSwerveModule.SteerRequestType steerRequestType) {
        this.SteerRequestType = steerRequestType;
        return this;
    }
}
