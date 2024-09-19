// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import frc.lib.six.drive.DrivetrainConfiguration;
import frc.lib.six.drive.swerve.SixSwerveDrivetrain;
import frc.lib.six.drive.swerve.SwerveModuleConfiguration;
import frc.lib.six.drive.swerve.requests.SixSwerveRequest;
import frc.lib.six.gyro.SixPigeon2;
import frc.lib.six.pid.SixPIDConstants;

import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class Jerry extends SixSwerveDrivetrain {
    // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
    // you trust your various sensors. Smaller numbers will cause the filter to
    // "trust" the estimate from that particular component more than the others.
    // This in turn means the particualr component will have a stronger influence
    // on the final pose estimate.

    /**
     * Standard deviations of model states. Increase these numbers to trust your
     * model's state estimates less. This
     * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
     * meters.
     */
    private static final Vector<N3> m_stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

    /**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
     * radians.
     */
    private static final Vector<N3> m_visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1,
        Units.degreesToRadians(25));

    private static final SixPIDConstants DRIVE_PID = new SixPIDConstants(0.0001).withkV(0.000169);
    private static final SixPIDConstants STEER_PID = new SixPIDConstants(0.05);

    private static final String CAN_BUS = "rio";

    private static final double MAX_VELOCITY = 4.0;
    private static final double MAX_ACCEL = 4.0;

    // distance from the right to left wheels on the robot
    private static final double TRACK_WIDTH = 26.5;
    // distance from the front to back wheels on the robot
    private static final double WHEEL_BASE = 26.5;

    // wheel diameter
    private static final double WHEEL_DIAMETER = 3.0;

    // Inverted
    private static final boolean RIGHT_SIDE_INVERTED = false;
    private static final boolean LEFT_SIDE_INVERTED = false;
    private static final boolean STEER_INVERTED = false;

    // Ratios
    private static final double DRIVE_RATIO = 5.076923077;
    private static final double STEER_RATIO = 46.41;
    // TODO: Find correct value. The back modules are both a bit off

    // private Field2d m_field = new Field2d();

    // MODULES
    private static final int FL_DRIVE_ID = 1;
    private static final int FL_STEER_ID = 5;
    private static final Rotation2d FL_OFFSET = Rotation2d.fromDegrees(113.1);
    private static final Translation2d FL_LOCATION = new Translation2d(Units.inchesToMeters(WHEEL_BASE) / 2,
        Units.inchesToMeters(TRACK_WIDTH) / 2);

    private static final int FR_DRIVE_ID = 2;
    private static final int FR_STEER_ID = 6;
    private static final Rotation2d FR_OFFSET = Rotation2d.fromDegrees(325.6);
    private static final Translation2d FR_LOCATION = new Translation2d(Units.inchesToMeters(WHEEL_BASE) / 2,
        Units.inchesToMeters(-TRACK_WIDTH) / 2);

    private static final int BL_DRIVE_ID = 3;
    private static final int BL_STEER_ID = 7;
    private static final Rotation2d BL_OFFSET = Rotation2d.fromDegrees(328.5);
    private static final Translation2d BL_LOCATION = new Translation2d(Units.inchesToMeters(-WHEEL_BASE) / 2,
        Units.inchesToMeters(TRACK_WIDTH) / 2);

    private static final int BR_DRIVE_ID = 4;
    private static final int BR_STEER_ID = 8;
    private static final Rotation2d BR_OFFSET = Rotation2d.fromDegrees(26.0);
    private static final Translation2d BR_LOCATION = new Translation2d(Units.inchesToMeters(-WHEEL_BASE) / 2,
        Units.inchesToMeters(-TRACK_WIDTH) / 2);

    private static final int STEER_CURRENT_LIMIT = 15;
    private static final int DRIVE_SUPPLY_LIMIT = 60;
    private static final int DRIVE_STATOR_LIMIT = 0;

    private final static SixPigeon2 m_gyro = new SixPigeon2(20);

    private static final DrivetrainConfiguration DRIVE_CONFIG = new DrivetrainConfiguration(
        DRIVE_PID, STEER_PID, STEER_CURRENT_LIMIT, DRIVE_SUPPLY_LIMIT, DRIVE_STATOR_LIMIT, CAN_BUS, MAX_VELOCITY, 2 * Math.PI, MAX_ACCEL, TRACK_WIDTH, WHEEL_BASE, WHEEL_DIAMETER, DRIVE_RATIO, STEER_RATIO);

    private static SwerveModuleConfiguration m_frontLeftConfig = new SwerveModuleConfiguration(
        FL_OFFSET,
        FL_LOCATION,
        LEFT_SIDE_INVERTED,
        STEER_INVERTED,
        DRIVE_CONFIG);

    private static SwerveModuleConfiguration m_frontRightConfig = new SwerveModuleConfiguration(
        FR_OFFSET,
        FR_LOCATION,
        RIGHT_SIDE_INVERTED,
        STEER_INVERTED,
        DRIVE_CONFIG);

    private static SwerveModuleConfiguration m_backLeftConfig = new SwerveModuleConfiguration(
        BL_OFFSET,
        BL_LOCATION,
        LEFT_SIDE_INVERTED,
        STEER_INVERTED,
        DRIVE_CONFIG);

    private static SwerveModuleConfiguration m_backRightConfig = new SwerveModuleConfiguration(
        BR_OFFSET,
        BR_LOCATION,
        RIGHT_SIDE_INVERTED,
        STEER_INVERTED,
        DRIVE_CONFIG);

    public Jerry() {
        super(
            DRIVE_CONFIG,
            m_gyro);

        super.setup(
            new REVMaxSwerveModule(
                FL_DRIVE_ID,
                FL_STEER_ID,
                m_frontLeftConfig),
            new REVMaxSwerveModule(
                FR_DRIVE_ID,
                FR_STEER_ID,
                m_frontRightConfig),
            new REVMaxSwerveModule(
                BL_DRIVE_ID,
                BL_STEER_ID,
                m_backLeftConfig),
            new REVMaxSwerveModule(
                BR_DRIVE_ID,
                BR_STEER_ID,
                m_backRightConfig) //
            );

        m_odom = new SwerveDrivePoseEstimator(
            m_kinematics,
            getGyroRotation2d(),
            getModulePositions(),
            new Pose2d(),
            m_stateStdDevs,
            m_visionMeasurementStdDevs);
    }

    @Override
    public void periodic() {
        super.periodic();

        // m_field.setRobotPose(getPoseMeters());
        // SmartDashboard.putData(m_field);

        SmartDashboard.putNumber("Pitch", getGyroPitchRotation2d().getDegrees());

        SmartDashboard.putNumber("FL angle", Math.toDegrees(m_modules.get(0).getAbsoluteEncoderRadians()));
        SmartDashboard.putNumber("FR angle", Math.toDegrees(m_modules.get(1).getAbsoluteEncoderRadians()));
        SmartDashboard.putNumber("BL angle", Math.toDegrees(m_modules.get(2).getAbsoluteEncoderRadians()));
        SmartDashboard.putNumber("BR angle", Math.toDegrees(m_modules.get(3).getAbsoluteEncoderRadians()));

        SmartDashboard.putNumber("FL velocity", Math.toDegrees(m_modules.get(0).getState().speedMetersPerSecond));
        SmartDashboard.putNumber("FR velocity", Math.toDegrees(m_modules.get(1).getState().speedMetersPerSecond));
        SmartDashboard.putNumber("BL velocity", Math.toDegrees(m_modules.get(2).getState().speedMetersPerSecond));
        SmartDashboard.putNumber("BR velocity", Math.toDegrees(m_modules.get(3).getState().speedMetersPerSecond));

        SmartDashboard.putNumber("X (meters)", m_odom.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Y (meters)", m_odom.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Heading (deg)", getHeading());

        SmartDashboard.putNumber("Velocity", super.getForwardVelocity());
    }

    public Command applyRequest(Supplier<SixSwerveRequest> request) {
        return run(() -> setControl(request.get()));
    }

    @Override
    public void simulationPeriodic() {
        updateSimState(0.020, RobotController.getBatteryVoltage());
    }
}