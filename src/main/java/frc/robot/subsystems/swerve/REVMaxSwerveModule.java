// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import frc.lib.six.drive.swerve.SixSwerveModule;
import frc.lib.six.drive.swerve.SwerveModuleConfiguration;
import frc.lib.six.encoder.SixSparkMAXEncoder;
import frc.lib.six.motor.SixSparkMAX;
import frc.lib.six.motor.configs.SixClosedLoopConfigs;
import frc.lib.six.motor.configs.SixClosedLoopConfigs.FeedbackSensor;

/** Add your docs here. */
public class REVMaxSwerveModule extends SixSwerveModule {
    private SixClosedLoopConfigs m_closedLoop = new SixClosedLoopConfigs().withFeedbackSource(FeedbackSensor.ConnectedRelative).withWrap(false);

    public REVMaxSwerveModule(
            int driveMotorPort,
            int steerMotorPort,
            SwerveModuleConfiguration config) {
        super(config);

        SixSparkMAX driveMotor = new SixSparkMAX(driveMotorPort);
        SixSparkMAX steerMotor = new SixSparkMAX(steerMotorPort);

        steerMotor.applyConfig(m_closedLoop);
        SixSparkMAXEncoder encoder = new SixSparkMAXEncoder(steerMotor.getAbsoluteEncoder());

        encoder.setCWPositive(false);
        
        driveMotor.restoreFactoryDefaults();
        steerMotor.restoreFactoryDefaults();

        super.setup(driveMotor, steerMotor, encoder);
    }
}
