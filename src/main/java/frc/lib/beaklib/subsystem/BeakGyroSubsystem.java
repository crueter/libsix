// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.subsystem;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.beaklib.gyro.BeakGyro;

/** A subsystem containing a gyro, with simulation support. */
public class BeakGyroSubsystem extends SubsystemBase {
    protected BeakGyro m_gyro;
    protected AnalogGyroSim m_gyroSim;

    public BeakGyroSubsystem() {
    }
    /**
     * Gets the gyro's reported angle.
     * 
     * @return A {@link Rotation2d} containing the reported angle of the gyro.
     */
    public Rotation2d getGyroRotation2d() {
        if (RobotBase.isSimulation()) {
            return Rotation2d.fromDegrees(m_gyroSim.getAngle());
        } else {
            return m_gyro.getYawRotation2d(true).Value;
        }
    }

    /**
     * Get the gyro's reported heading.
     * 
     * @return The reported angle of the gyro in degrees.
     */
    public double getGyroHeading() {
        return getGyroRotation2d().getDegrees();
    }

    /**
     * Get the gyro's reported rate.
     * 
     * @return The reported rate of the gyro in degrees per second.
     */
    public double getGyroRate() {
        if (RobotBase.isSimulation()) {
            return m_gyroSim.getRate();
        } else {
            return m_gyro.getAngularVelocity().Value.in(DegreesPerSecond);
        }
    }

    /**
     * Get the gyro's reported pitch.
     * 
     * @return The reported pitch of the gyro.
     */
    public Rotation2d getGyroPitchRotation2d() {
        return m_gyro.getPitchRotation2d(true).Value;
    }

    /**
     * Get the gyro's reported roll.
     * 
     * @return The reported roll of the gyro.
     */
    public Rotation2d getGyroRollRotation2d() {
        return m_gyro.getRollRotation2d(true).Value;
    }
}
