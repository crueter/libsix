// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.drive.swerve.requests;

import java.util.List;

import frc.lib.beaklib.drive.swerve.BeakSwerveModule;

/** Do nothing. */
public class BeakSwerveIdle implements BeakSwerveRequest {

    @Override
    public void apply(SwerveControlRequestParameters parameters, List<BeakSwerveModule> modules) {

    }
}
