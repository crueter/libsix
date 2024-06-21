// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.drive.swerve;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.beaklib.drive.BeakDrivetrain;
import frc.lib.beaklib.drive.swerve.BeakSwerveModule.DriveRequestType;
import frc.lib.beaklib.drive.swerve.requests.BeakChassisSpeedsDrive;
import frc.lib.beaklib.drive.swerve.requests.BeakSwerveIdle;
import frc.lib.beaklib.drive.swerve.requests.BeakSwerveRequest;
import frc.lib.beaklib.drive.swerve.requests.BeakSwerveRequest.SwerveControlRequestParameters;
import frc.lib.beaklib.gyro.BeakGyro;
import frc.lib.beaklib.gyro.BeakV6Pigeon2;

/** Generic Swerve Drivetrain subsystem. */
public class BeakSwerveDrivetrain extends BeakDrivetrain {
    /**
     * <p>
     * The modules in this swerve drivetrain.
     * </p>
     * These are in the same order as passed in the constructor; i.e if the front
     * left
     * module is passed in as the first module, <code>m_modules.get(0)</code> would
     * return the front left module.
     */
    protected List<BeakSwerveModule> m_modules = new ArrayList<BeakSwerveModule>();

    protected int m_numModules;

    protected SwerveDrivePoseEstimator m_odom;
    protected SwerveDriveKinematics m_kinematics;

    protected BeakChassisSpeedsDrive m_chassisSpeedsDrive = new BeakChassisSpeedsDrive().withDriveRequestType(DriveRequestType.Velocity);

    protected BeakSwerveRequest m_currentRequest = new BeakSwerveIdle();
    protected SwerveControlRequestParameters m_requestParameters = new SwerveControlRequestParameters();

    protected BeakSwerveSim m_simDrive;

    /**
     * Create a new Swerve drivetrain.
     * 
     * @param gyro
     *             The gyroscope used by this drivetrain.
     */
    public BeakSwerveDrivetrain(
            DrivetrainConfiguration config,
            BeakGyro gyro) {
        super(config);

        m_gyro = gyro;
    }

    public void setup(BeakSwerveModule... modules) {
        m_numModules = modules.length;
        Translation2d[] moduleLocations = new Translation2d[m_numModules];

        for (int i = 0; i < m_numModules; i++) {
            BeakSwerveModule module = modules[i];
            m_modules.add(module);
            moduleLocations[i] = module.Config.ModuleLocation;
        }

        m_kinematics = new SwerveDriveKinematics(moduleLocations);

        m_odom = new SwerveDrivePoseEstimator(m_kinematics, getGyroRotation2d(), getModulePositions(), new Pose2d());

        m_requestParameters.kinematics = m_kinematics;
        m_requestParameters.swervePositions = getModuleLocations();
        m_requestParameters.updatePeriod = 1.0 / 50.0;

        // m_simDrive = new BeakSwerveSim(getModuleLocations(), (BeakV6Pigeon2) m_gyro, m_config, m_modules);

        resetSteering();
    }

    /**
     * Set the next requested control type.
     * 
     * @param request The {@link BeakSwerveRequest} to apply.
     */
    public void setControl(BeakSwerveRequest request) {
        m_currentRequest = request;
    }

    @Override
    public Pose2d updateOdometry() {
        m_pose = m_odom.updateWithTime(
                RobotController.getFPGATime() / 1000000.,
                getGyroRotation2d(),
                getModulePositions());

        return m_pose;
    }

    @Override
    public void addVisionMeasurement(Pose2d estimatedPose, double timestamp) {
        Transform2d poseError = estimatedPose.minus(m_odom.getEstimatedPosition());

        if (!estimatedPose.equals(new Pose2d()) && !estimatedPose.equals(getPoseMeters()) &&
                Math.abs(poseError.getX()) < 0.5 &&
                Math.abs(poseError.getY()) < 0.5) {
            m_odom.addVisionMeasurement(estimatedPose, timestamp);
        }
    }

    @Override
    public Pose2d getPoseMeters() {
        return m_odom.getEstimatedPosition();
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        if (!pose.equals(new Pose2d()))
            m_odom.resetPosition(getGyroRotation2d(), getModulePositions(), pose);
    }

    @Override
    public void drive(ChassisSpeeds speeds) {
        setControl(m_chassisSpeedsDrive.withSpeeds(speeds));
        SmartDashboard.putNumber("X speed", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Y speed", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Omega speed", speeds.omegaRadiansPerSecond);
    }

    /**
     * Updates all the simulation state variables for this
     * drivetrain class. User provides the update variables for the simulation.
     *
     * @param dtSeconds     time since last update call
     * @param supplyVoltage voltage as seen at the motor controllers
     */
    public void updateSimState(double dtSeconds, double supplyVoltage) {
        m_simDrive.update(dtSeconds, supplyVoltage, m_modules);
    }

    /* Swerve-specific Methods */

    /**
     * Get the states of each module.
     * 
     * @return Array of {@link SwerveModuleState}s for each module.
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[m_numModules];
        for (int i = 0; i < m_numModules; i++) {
            states[i] = m_modules.get(i).getState();
        }

        return states;
    }

    /**
     * Get the positions of each module.
     * 
     * @return Array of {@link SwerveModulePosition}s for each module.
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[m_numModules];
        for (int i = 0; i < m_numModules; i++) {
            states[i] = m_modules.get(i).getPosition();
        }

        return states;
    }

    /**
     * Get the locations of each module.
     * 
     * @return Array of {@link Translation2d}s for each module.
     */
    public Translation2d[] getModuleLocations() {
        Translation2d[] locations = new Translation2d[m_numModules];
        for (int i = 0; i < m_numModules; i++) {
            locations[i] = m_modules.get(i).Config.ModuleLocation;
        }

        return locations;
    }

    /**
     * Get the angles of each module.
     * 
     * @return Array of the angles for each module.
     */
    public double[] getModuleAngles() {
        double[] states = new double[m_numModules];
        for (int i = 0; i < m_numModules; i++) {
            states[i] = Units.radiansToDegrees(m_modules.get(i).getSteerEncoderRadians());
        }

        return states;
    }

    /**
     * Reset all drive and steer encoders to zero.
     */
    public void resetEncoders() {
        for (BeakSwerveModule module : m_modules) {
            module.resetEncoders();
        }
    }

    /**
     * Re-zero all steer encoders to match the CANCoder.
     */
    public void resetSteering() {
        for (BeakSwerveModule module : m_modules) {
            module.resetSteerMotor();
        }
    }

    /**
     * Zero the pose and heading of the robot.
     */
    public void zero() {
        resetSteering();
        m_odom.resetPosition(getGyroRotation2d(), getModulePositions(), new Pose2d());
    }

    private ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(
                getModuleStates());
    }

    /**
     * Get the forward velocity of the drivetrain.
     * 
     * @return The X (forward) velocity of the drivetrain, in meters per second.
     */
    public double getForwardVelocity() {
        return getChassisSpeeds().vxMetersPerSecond;
    }

    /**
     * Get the sideways velocity of the drivetrain.
     * 
     * @return The Y (sideways) velocity of the drivetrain, in meters per second.
     */
    public double getSidewaysVelocity() {
        return getChassisSpeeds().vyMetersPerSecond;
    }

    /**
     * Get the angular velocity of the drivetrain.
     * 
     * @return The angular velocity of the drivetrain, in radians per second.
     */
    public double getAngularVelocity() {
        return getChassisSpeeds().omegaRadiansPerSecond;
    }

    public boolean isHolonomic() {
        return true;
    }

    @Override
    public void periodic() {
        super.periodic();

        updateOdometry();

        m_requestParameters.currentPose = m_odom.getEstimatedPosition();
        m_requestParameters.currentChassisSpeed = getChassisSpeeds();
        m_requestParameters.timestamp = Timer.getFPGATimestamp();

        // if (m_currentRequest != null) {
        m_currentRequest.apply(m_requestParameters, m_modules);
        // }
    }
}
