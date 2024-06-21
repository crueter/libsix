// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.drive.swerve.requests;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.beaklib.drive.swerve.BeakSwerveModule;
import frc.lib.beaklib.drive.swerve.BeakSwerveModule.DriveRequestType;

/** Drive based upon field-relative velocities. */
public class BeakXDrive implements BeakSwerveRequest {
    /**
     * The type of control request to use for the steer motor.
     */
    public BeakSwerveModule.SteerRequestType SteerRequestType = BeakSwerveModule.SteerRequestType.Position;

    /**
     * The last applied state in case we don't have anything to drive.
     */
    protected SwerveModuleState[] m_lastAppliedState = null;

    // TODO: look forward based on acceleration
    public void apply(SwerveControlRequestParameters parameters, List<BeakSwerveModule> modulesToApply) {
        for (int i = 0; i < modulesToApply.size(); ++i) {
            Rotation2d angle = modulesToApply.get(i).Config.ModuleLocation.getAngle();
            modulesToApply.get(i).apply(new SwerveModuleState(0, angle), DriveRequestType.Voltage, SteerRequestType);
        }
    }

    /**
     * Sets the type of control request to use for the steer motor.
     *
     * @param steerRequestType The type of control request to use for the steer
     *                         motor
     * @return this request
     */
    public BeakXDrive withSteerRequestType(BeakSwerveModule.SteerRequestType steerRequestType) {
        this.SteerRequestType = steerRequestType;
        return this;
    }
}
