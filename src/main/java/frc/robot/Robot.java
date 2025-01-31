// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    // private SixSparkMAX m_motor = new SixSparkMAX(6);
    // private SixSparkMAXEncoder m_encoder = new
    // SixSparkMAXEncoder(m_motor.getAbsoluteEncoder());
    // private DataSignal<Double> m_deez = m_motor.getPositionNU(false);

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
        // m_motor.restoreFactoryDefaults();
        // m_motor.setPID(new SixPIDConstants(0.05));
        // m_motor.getPIDController().setFeedbackDevice(m_motor.getEncoder());
        // m_motor.getPIDController().setPositionPIDWrappingEnabled(false);
        // m_motor.setEncoderGearRatio(49.0);
        // m_deez.setUpdateFrequency(50);

        // m_encoder.setAbsoluteOffset(Rotation2d.fromDegrees(325.6));
        // m_encoder.setCWPositive(true);
        // m_motor.setEncoderPositionMotorRotations(m_encoder.getEncoderPosition(true).getValue().getRotations());
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        CommandScheduler.getInstance().run();
        // SmartDashboard.putNumber("SNEED", m_motor.getEncoder().getPosition());
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        // m_motor.getPIDController().setReference(49 * 0., ControlType.kPosition);

        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        // m_motor.setEncoderPositionNU(0);

    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        REVPhysicsSim.getInstance().run();
    }
}
