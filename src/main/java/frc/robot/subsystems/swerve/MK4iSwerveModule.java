// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import frc.lib.six.drive.swerve.SixSwerveModule;
import frc.lib.six.drive.swerve.SwerveModuleConfiguration;
import frc.lib.six.encoder.SixCANCoder;
import frc.lib.six.motor.SixTalonFX;

/** Add your docs here. */
public class MK4iSwerveModule extends SixSwerveModule {
    public MK4iSwerveModule(
        int driveMotorPort,
        int steerMotorPort,
        int encoderPort,
        SwerveModuleConfiguration config) {
        super(config);

        SixTalonFX driveMotor = new SixTalonFX(driveMotorPort, config.DriveConfig.CANBus);
        SixTalonFX steerMotor = new SixTalonFX(steerMotorPort, config.DriveConfig.CANBus);

        SixCANCoder steerEncoder = new SixCANCoder(encoderPort, config.DriveConfig.CANBus);

        super.setup(driveMotor, steerMotor, steerEncoder);
    }
}
