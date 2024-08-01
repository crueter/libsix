// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.six.drive.swerve.requests;

import java.util.List;

import frc.lib.six.drive.swerve.SixSwerveModule;

/** Do nothing. */
public class SixSwerveIdle implements SixSwerveRequest {

    @Override
    public void apply(SwerveControlRequestParameters parameters, List<SixSwerveModule> modules) {

    }
}
