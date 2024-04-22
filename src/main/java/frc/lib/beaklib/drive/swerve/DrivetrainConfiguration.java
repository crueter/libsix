// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.drive.swerve;

import frc.lib.beaklib.pid.BeakPIDConstants;

/** Constants of any drivetrain type. */
public class DrivetrainConfiguration {
    public final BeakPIDConstants DrivePID;
    public final BeakPIDConstants TurnPID;

    public final int TurnCurrentLimit;
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
    public final double TurnRatio;

    /**
     * @param drivePID           PID constants for the drive motors
     * @param turnPID            PID constants for the turn motors
     * @param turnCurrentLimit   Supply/stator limits for the turn motors
     * @param driveSupplyLimit   Supply limits for the drive motors
     * @param driveStatorLimit   Stator limits for the drive motors
     * @param canBus             The CAN Bus the drivetrain is attached to.
     * @param maxSpeed           The robot's max speed, in m/s
     * @param maxAngularVelocity The robot's max angular velocity, in rad/s
     * @param maxAcceleration    The robot's max acceleration, in m/s/s
     * @param trackWidth         Track width of the robot, in inches
     * @param wheelBase          Wheel base of the robot, in inches
     * @param wheelDiameter      Diameter of the wheels, in inches
     * @param driveRatio         Ratio between the drive motor & output wheel (>1)
     * @param turnRatio          Ratio between the turn motor & output bearing (>1)
     */
    public DrivetrainConfiguration(BeakPIDConstants drivePID, BeakPIDConstants turnPID, int turnCurrentLimit,
            int driveSupplyLimit, int driveStatorLimit, String canBus, double maxSpeed, double maxAngularVelocity,
            double maxAcceleration, double trackWidth, double wheelBase, double wheelDiameter, double driveRatio,
            double turnRatio) {
        DrivePID = drivePID;
        TurnPID = turnPID;
        TurnCurrentLimit = turnCurrentLimit;
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
        TurnRatio = turnRatio;
    }
}
