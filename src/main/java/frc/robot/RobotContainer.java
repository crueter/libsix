// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.lib.beaklib.BeakXBoxController;
import frc.lib.beaklib.Util;
import frc.lib.beaklib.drive.swerve.BeakSwerveDrivetrain;
import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    // private final BeakSwerveDrivetrain m_drive;
    private final Drivetrain m_drive;

    // Controller
    private final BeakXBoxController m_driverController = new BeakXBoxController(0);

    // Limiters, etc.
    private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(4.0);
    private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(4.0);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(4.0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // m_drive = new SwerveDrivetrain();
        m_drive = new Drivetrain();

        // Configure the button bindings
        configureButtonBindings();
        initAutonChooser();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // ==================
        // DEFAULT COMMANDS
        // ==================
        m_drive.setDefaultCommand(
                new RunCommand(() -> m_drive.drive(
                        -speedScaledDriverLeftY(),
                        speedScaledDriverLeftX(),
                        speedScaledDriverRightX(),
                        true),
                        m_drive));

        // ================================================
        // DRIVER CONTROLLER - START
        // ZERO DRIVETRAIN
        // ================================================
        // m_driverController.start.onTrue(new InstantCommand(m_drive::zero));
    }

    private void initAutonChooser() {
    }

    public double speedScaledDriverLeftY() {
        return m_yLimiter.calculate(-Util.speedScale(m_driverController.getLeftYAxis(),
                getCurrentSpeedScale(),
                m_driverController.getRightTrigger()));
    }

    public double speedScaledDriverRightX() {
        return m_rotLimiter.calculate(-Util.speedScale(m_driverController.getRightXAxis(),
                getCurrentSpeedScale(),
                m_driverController.getRightTrigger()));
    }

    public double speedScaledDriverLeftX() {
        return m_xLimiter.calculate(-Util.speedScale(m_driverController.getLeftXAxis(),
                getCurrentSpeedScale(),
                m_driverController.getRightTrigger()));
    }

    private double getCurrentSpeedScale() {
        return 0.25;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
