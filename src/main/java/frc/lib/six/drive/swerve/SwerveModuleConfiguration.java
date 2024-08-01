// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.six.drive.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.six.drive.DrivetrainConfiguration;

/** Class containing general configuration for a {@link SixSwerveModule}. */
public class SwerveModuleConfiguration {
    public final Rotation2d AngleOffset;
    public final Translation2d ModuleLocation;

    public final boolean DriveInverted;
    public final boolean SteerInverted;

    public final DrivetrainConfiguration DriveConfig;

    /**
     * @param angleOffset
     *            Zero offset of this module.
     * @param moduleLocation
     *            Translation from the center of the robot to this module.
     * @param driveInverted
     *            Whether or not the drive motor is inverted.
     * @param steerInverted
     *            Whether or not the steer motor is inverted.
     * @param driveConfig
     *            {@link DrivetrainConfiguration} of the drivetrain this
     *            module is on.
     */
    public SwerveModuleConfiguration(
        Rotation2d angleOffset,
        Translation2d moduleLocation,
        boolean driveInverted,
        boolean steerInverted,
        DrivetrainConfiguration driveConfig) {
        this.DriveConfig = driveConfig;

        this.AngleOffset = angleOffset;
        this.ModuleLocation = moduleLocation;

        this.DriveInverted = driveInverted;
        this.SteerInverted = steerInverted;
    }
}
