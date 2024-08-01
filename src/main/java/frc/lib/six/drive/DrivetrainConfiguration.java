// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.six.drive;

import frc.lib.six.pid.SixPIDConstants;

/** Constants of any drivetrain type. */
public class DrivetrainConfiguration {
    public final SixPIDConstants DrivePID;
    public final SixPIDConstants SteerPID;

    public final int SteerCurrentLimit;
    public final int DriveSupplyLimit;

    public final int DriveStatorLimit;

    public final String CANBus;

    public final double MaxSpeed;
    public final double MaxAngularVelocity;
    public final double MaxAcceleration;

    public final double TrackWidth;
    public final double WheelBase;
    public final double WheelDiameter;

    public final double DriveRatio;
    public final double SteerRatio;

    /**
     * @param drivePID           PID constants for the drive motors
     * @param steerPID            PID constants for the steer motors
     * @param steerCurrentLimit   Supply/stator limits for the steer motors
     * @param driveSupplyLimit   Supply limits for the drive motors
     * @param driveStatorLimit   Stator limits for the drive motors
     * @param canBus             The CAN Bus the drivetrain is attached to.
     * @param maxSpeed           The robot's max speed, in m/s
     * @param maxAngularVelocity The robot's max angular velocity, in rad/s
     * @param maxAcceleration    The robot's max acceleration, in m/s/s
     * @param trackWidth         Track width of the robot, in inches
     * @param wheelBase          Wheel base of the robot, in inches
     * @param wheelDiameter      Diameter of the wheels, in inches
     * @param driveRatio         Ratio between the drive motor and output wheel (>1)
     * @param steerRatio          Ratio between the steer motor and output bearing (>1)
     */
    public DrivetrainConfiguration(SixPIDConstants drivePID, SixPIDConstants steerPID, int steerCurrentLimit,
            int driveSupplyLimit, int driveStatorLimit, String canBus, double maxSpeed, double maxAngularVelocity,
            double maxAcceleration, double trackWidth, double wheelBase, double wheelDiameter, double driveRatio,
            double steerRatio) {
        DrivePID = drivePID;
        SteerPID = steerPID;
        SteerCurrentLimit = steerCurrentLimit;
        DriveSupplyLimit = driveSupplyLimit;
        DriveStatorLimit = driveStatorLimit;
        CANBus = canBus;
        MaxSpeed = maxSpeed;
        MaxAngularVelocity = maxAngularVelocity;
        MaxAcceleration = maxAcceleration;
        TrackWidth = trackWidth;
        WheelBase = wheelBase;
        WheelDiameter = wheelDiameter;
        DriveRatio = driveRatio;
        SteerRatio = steerRatio;
    }
}
