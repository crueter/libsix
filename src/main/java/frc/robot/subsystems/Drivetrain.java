// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.beaklib.drive.BeakDifferentialDrivetrain;
import frc.lib.beaklib.drive.swerve.DrivetrainConfiguration;
import frc.lib.beaklib.gyro.BeakNavX;
import frc.lib.beaklib.motor.BeakMotorControllerGroup;
import frc.lib.beaklib.motor.BeakTalonSRX;
import frc.lib.beaklib.pid.BeakPIDConstants;

public class Drivetrain extends BeakDifferentialDrivetrain {
    private static final double kP = 0.05;
    private static final double kD = 0.0;

    private static final BeakPIDConstants DRIVE_PID = new BeakPIDConstants(kP, 0., kD);

    private static final int FL_ID = 1;
    private static final int BL_ID = 2;
    private static final int FR_ID = 3;
    private static final int BR_ID = 4;

    private static final double MAX_VELOCITY = 4.5;

    // distance from the right to left wheels on the robot
    private static final double TRACK_WIDTH = 24;
    // distance from the front to back wheels on the robot
    private static final double WHEEL_BASE = 25;

    private static final double WHEEL_DIAMETER = 6.0;
    private static final double GEAR_RATIO = 7.5;

    private final Field2d field;

    // private static final SimpleMotorFeedforward FEED_FORWARD = new
    // SimpleMotorFeedforward(
    // 1.1161,
    // .17294,
    // 0.087223);

    private final DifferentialDrivetrainSim sim = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDualCIMPerSide,
            KitbotGearing.k7p31, KitbotWheelSize.kSixInch, null);

    private static final int DRIVE_SUPPLY_LIMIT = 60;

    private static final DrivetrainConfiguration CONFIG = new DrivetrainConfiguration(
            DRIVE_PID, DRIVE_PID, 0, DRIVE_SUPPLY_LIMIT, 0, "rio", MAX_VELOCITY, Math.PI, MAX_VELOCITY, TRACK_WIDTH,
            WHEEL_BASE,
            WHEEL_DIAMETER, GEAR_RATIO, 0.0);

    private final BeakNavX m_gyro = new BeakNavX(SPI.Port.kMXP);

    private final BeakTalonSRX m_FL, m_BL, m_FR, m_BR;
    private final BeakMotorControllerGroup m_left;
    private final BeakMotorControllerGroup m_right;

    /** Creates a new Drivetrain. */
    public Drivetrain() {
        super(CONFIG);

        m_FL = new BeakTalonSRX(FL_ID);
        m_FR = new BeakTalonSRX(FR_ID);
        m_BL = new BeakTalonSRX(BL_ID);
        m_BR = new BeakTalonSRX(BR_ID);

        m_left = new BeakMotorControllerGroup(m_FL, m_BL);
        m_right = new BeakMotorControllerGroup(m_FR, m_BR);

        m_right.setInverted(true);

        super.setup(m_right, m_left, m_gyro);

        field = new Field2d();
    }

    @Override
    public void drive(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);

        drive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);

        sim.setInputs(wheelSpeeds.leftMetersPerSecond * 12.0 / MAX_VELOCITY,
                wheelSpeeds.rightMetersPerSecond * 12.0 / MAX_VELOCITY);
    }

    @Override
    public void periodic() {
        super.updateOdometry();
    }

    @Override
    public void simulationPeriodic() {
        sim.update(0.020);
        field.setRobotPose(sim.getPose());
        SmartDashboard.putData("Field", field);
    }
}