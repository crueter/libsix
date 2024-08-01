// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.six.pid;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/** Class representing PID or PIDF constants. */
public class SixPIDConstants {
    public double kP;
    public double kI;
    public double kD;
    /** This can also be represented as kV. */
    public double kV;
    public double kS;
    public double period;

    private SimpleMotorFeedforward m_feedforward;
    
    public SixPIDConstants(double kP, double kI, double kD, double kV, double kS, double period) {
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.kV = kV;
      this.kS = kS;
      this.period = period;
    }
  
    public SixPIDConstants(double kP, double kI, double kD, double kV, double kS) {
      this(kP, kI, kD, kV, kS, 0.02);
    }

    public SixPIDConstants(double kP, double kI, double kD, double kV) {
        this(kP, kI, kD, kV, 0, 0.02);
    }
    public SixPIDConstants(double kP, double kI, double kD) {
        this(kP, kI, kD, 0, 0, 0.02);
    }

    public SixPIDConstants(double kP) {
        this(kP, 0, 0, 0, 0.0, 0.02);
    }

    public SixPIDConstants() {
        this(0, 0, 0, 0, 0.0, 0.02);
    }

    public SixPIDConstants withkP(double kP) {
        this.kP = kP;
        return this;
    }

    public SixPIDConstants withkI(double kI) {
        this.kI = kI;
        return this;
    }

    public SixPIDConstants withkD(double kD) {
        this.kD = kD;
        return this;
    }

    public SixPIDConstants withkV(double kV) {
        this.kV = kV;
        return this;
    }

    public SixPIDConstants withkS(double kS) {
        this.kS = kS;
        return this;
    }

    /** Grab PIDF constants from a CTRE Phoenix MC PID slot configuration. */
    public SixPIDConstants(SlotConfiguration phoenixSlotConfig) {
        this(phoenixSlotConfig.kP, phoenixSlotConfig.kI, phoenixSlotConfig.kD, phoenixSlotConfig.kF, 0, 0.02);
    }

    public double getFeedForward(double velocity) {
        return m_feedforward.calculate(velocity);
    }
  }
  