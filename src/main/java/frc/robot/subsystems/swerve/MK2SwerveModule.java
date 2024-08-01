// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import frc.lib.six.drive.swerve.SixSwerveModule;
import frc.lib.six.drive.swerve.SwerveModuleConfiguration;
import frc.lib.six.encoder.SixAnalogInput;
import frc.lib.six.motor.SixSparkMAX;

/** Add your docs here. */
public class MK2SwerveModule extends SixSwerveModule {
    public MK2SwerveModule(
        int driveMotorPort,
        int steerMotorPort,
        int encoderPort,
        SwerveModuleConfiguration config) {
        super(config);

        SixSparkMAX driveMotor = new SixSparkMAX(driveMotorPort);
        SixSparkMAX steerMotor = new SixSparkMAX(steerMotorPort);

        driveMotor.restoreFactoryDefaults();
        steerMotor.restoreFactoryDefaults();

        SixAnalogInput steerEncoder = new SixAnalogInput(encoderPort);

        super.setup(driveMotor, steerMotor, steerEncoder);
    }
}
