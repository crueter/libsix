// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.six.drive.swerve.requests;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.six.drive.swerve.SixSwerveModule;

/** Base interface for swerve requests. */
public interface SixSwerveRequest {
    public class SwerveControlRequestParameters {
        public SwerveDriveKinematics kinematics;
        public ChassisSpeeds currentChassisSpeed;
        public Pose2d currentPose;
        public double timestamp;
        public Translation2d[] swervePositions;
        public double updatePeriod;
    }

    public void apply(SwerveControlRequestParameters parameters, List<SixSwerveModule> modules);
}
