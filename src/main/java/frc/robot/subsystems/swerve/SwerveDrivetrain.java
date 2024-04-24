// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import frc.lib.beaklib.drive.swerve.BeakSwerveDrivetrain;
import frc.lib.beaklib.drive.swerve.DrivetrainConfiguration;
import frc.lib.beaklib.drive.swerve.SwerveModuleConfiguration;
import frc.lib.beaklib.gyro.BeakV6Pigeon2;
import frc.lib.beaklib.pid.BeakPIDConstants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class SwerveDrivetrain extends BeakSwerveDrivetrain {
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

    private static final BeakPIDConstants DRIVE_PID = new BeakPIDConstants(0.0, 0.0, 0.0, 0.0, 0.0);
    private static final BeakPIDConstants TURN_PID = new BeakPIDConstants(10.0).withkV(1.0);

    private static final int PIGEON2_ID = 0;
    private static final String CAN_BUS = "rio";

    // private static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(
    //     (.2212 + .151 + .163) / 3.,
    //     (2.3 + 2.32 + 2.33) / 3.,
    //     (.421 + .849 + .708) / 3.);

    private static final double MAX_VELOCITY = 6.0;
    private static final double MAX_ACCEL = 6.0;

    // distance from the right to left wheels on the robot
    private static final double TRACK_WIDTH = 19.688;
    // distance from the front to back wheels on the robot
    private static final double WHEEL_BASE = 23.688;

    // wheel diameter
    private static final double WHEEL_DIAMETER = 3.85;

    // Inverted
    private static final boolean RIGHT_SIDE_INVERTED = true;
    private static final boolean LEFT_SIDE_INVERTED = false;
    private static final boolean STEER_INVERTED = true;

    // Ratios
    private static final double DRIVE_RATIO = 1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
    private static final double TURN_RATIO = (150.0 / 7.0);

    private Field2d m_field = new Field2d();

    // MODULES
    private static final int FL_DRIVE_ID = 2;
    private static final int FL_TURN_ID = 1;
    private static final int FL_ENCODER_ID = 1;
    private static final Rotation2d FL_OFFSET = Rotation2d.fromDegrees(-270.0);//-357.4);
    private static final Translation2d FL_LOCATION = new Translation2d(Units.inchesToMeters(WHEEL_BASE) / 2,
        Units.inchesToMeters(TRACK_WIDTH) / 2);

    private static final int FR_DRIVE_ID = 4;
    private static final int FR_TURN_ID = 3;
    private static final int FR_ENCODER_ID = 2;
    private static final Rotation2d FR_OFFSET = Rotation2d.fromDegrees(90.0);//-271.3);
    private static final Translation2d FR_LOCATION = new Translation2d(Units.inchesToMeters(WHEEL_BASE) / 2,
        Units.inchesToMeters(-TRACK_WIDTH) / 2);

    private static final int BL_DRIVE_ID = 6;
    private static final int BL_TURN_ID = 5;
    private static final int BL_ENCODER_ID = 3;
    private static final Rotation2d BL_OFFSET = Rotation2d.fromDegrees(90.0);//-327.3);
    private static final Translation2d BL_LOCATION = new Translation2d(Units.inchesToMeters(-WHEEL_BASE) / 2,
        Units.inchesToMeters(TRACK_WIDTH) / 2);

    private static final int BR_DRIVE_ID = 8;
    private static final int BR_TURN_ID = 7;
    private static final int BR_ENCODER_ID = 4;
    private static final Rotation2d BR_OFFSET = Rotation2d.fromDegrees(-270.0);//-160.5);
    private static final Translation2d BR_LOCATION = new Translation2d(Units.inchesToMeters(-WHEEL_BASE) / 2,
        Units.inchesToMeters(-TRACK_WIDTH) / 2);

    private static final int TURN_CURRENT_LIMIT = 20;
    private static final int DRIVE_SUPPLY_LIMIT = 75;
    private static final int DRIVE_STATOR_LIMIT = 80;

    private final static BeakV6Pigeon2 m_gyro = new BeakV6Pigeon2(PIGEON2_ID, CAN_BUS);

    private static final DrivetrainConfiguration DRIVE_CONFIG = new DrivetrainConfiguration(
        DRIVE_PID, TURN_PID, TURN_CURRENT_LIMIT, DRIVE_SUPPLY_LIMIT, DRIVE_STATOR_LIMIT, CAN_BUS, MAX_VELOCITY, 2 * Math.PI, MAX_ACCEL, TRACK_WIDTH, WHEEL_BASE, WHEEL_DIAMETER, DRIVE_RATIO, TURN_RATIO);

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

    public SwerveDrivetrain() {
        super(
            DRIVE_CONFIG,
            m_gyro);

        super.setup(
            new MK4iSwerveModule(
                FL_DRIVE_ID,
                FL_TURN_ID,
                FL_ENCODER_ID,
                m_frontLeftConfig),
            new MK4iSwerveModule(
                FR_DRIVE_ID,
                FR_TURN_ID,
                FR_ENCODER_ID,
                m_frontRightConfig),
            new MK4iSwerveModule(
                BL_DRIVE_ID,
                BL_TURN_ID,
                BL_ENCODER_ID,
                m_backLeftConfig),
            new MK4iSwerveModule(
                BR_DRIVE_ID,
                BR_TURN_ID,
                BR_ENCODER_ID,
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

        m_field.setRobotPose(getPoseMeters());
        SmartDashboard.putData(m_field);

        SmartDashboard.putNumber("Pitch", getGyroPitchRotation2d().getDegrees());

        SmartDashboard.putNumber("FL angle", Math.toDegrees(m_modules.get(0).getAbsoluteEncoderRadians()));
        SmartDashboard.putNumber("FR angle", Math.toDegrees(m_modules.get(1).getAbsoluteEncoderRadians()));
        SmartDashboard.putNumber("BL angle", Math.toDegrees(m_modules.get(2).getAbsoluteEncoderRadians()));
        SmartDashboard.putNumber("BR angle", Math.toDegrees(m_modules.get(3).getAbsoluteEncoderRadians()));

        SmartDashboard.putNumber("X (meters)", m_odom.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Y (meters)", m_odom.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Heading (deg)", getHeading());

        SmartDashboard.putNumber("Velocity", super.getForwardVelocity());
    }

    @Override
    public void simulationPeriodic() {
        updateSimState(0.020, RobotController.getBatteryVoltage());
    }
}