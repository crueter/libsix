// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import frc.lib.beaklib.drive.swerve.BeakSwerveModule;
import frc.lib.beaklib.drive.swerve.SwerveModuleConfiguration;
import frc.lib.beaklib.encoder.BeakAnalogInput;
import frc.lib.beaklib.motor.BeakSparkMAX;

/** Add your docs here. */
public class MK2SwerveModule extends BeakSwerveModule {
    public MK2SwerveModule(
        int driveMotorPort,
        int steerMotorPort,
        int encoderPort,
        SwerveModuleConfiguration config) {
        super(config);

        BeakSparkMAX driveMotor = new BeakSparkMAX(driveMotorPort);
        BeakSparkMAX steerMotor = new BeakSparkMAX(steerMotorPort);

        driveMotor.restoreFactoryDefaults();
        steerMotor.restoreFactoryDefaults();

        BeakAnalogInput steerEncoder = new BeakAnalogInput(encoderPort);

        super.setup(driveMotor, steerMotor, steerEncoder);
    }
}
