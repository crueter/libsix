// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.motor;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.beaklib.motor.configs.BeakClosedLoopConfigs;
import frc.lib.beaklib.motor.configs.BeakCurrentLimitConfigs;
import frc.lib.beaklib.motor.configs.BeakDutyCycleConfigs;
import frc.lib.beaklib.motor.configs.BeakHardwareLimitSwitchConfigs;
import frc.lib.beaklib.motor.configs.BeakMotionProfileConfigs;
import frc.lib.beaklib.motor.configs.BeakVoltageConfigs;
import frc.lib.beaklib.motor.configs.BeakHardwareLimitSwitchConfigs.BeakLimitSwitchSource;
import frc.lib.beaklib.pid.BeakPIDConstants;

/** Common motor controller interface for Talon SRX. */
public class BeakTalonSRX extends WPI_TalonSRX implements BeakMotorController {
    private double m_velocityConversionConstant = 4096. / 600.;
    private double m_positionConversionConstant = 4096.;
    private double m_gearRatio = 1.;
    private Measure<Distance> m_wheelDiameter = Inches.of(4.);

    private int m_slot = 0;
    private double m_arbFeedforward = 0.;

    private DigitalInput m_dioRevLimitSwitch = null;
    private DigitalInput m_dioFwdLimitSwitch = null;

    private BeakLimitSwitchSource m_forwardSource = BeakLimitSwitchSource.None;
    private BeakLimitSwitchSource m_reverseSource = BeakLimitSwitchSource.None;

    public BeakTalonSRX(int port) {
        super(port);
    }

    public void setSlot(int slot) {
        super.selectProfileSlot(slot, 0);
        m_slot = slot;
    }

    @Override
    public void setBrake(boolean brake) {
        super.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public void setVelocityNU(double nu) {
        super.set(ControlMode.Velocity, nu, DemandType.ArbitraryFeedForward, m_arbFeedforward / 12.);
    }

    @Override
    public void setPositionNU(double nu) {
        super.set(ControlMode.Position, nu, DemandType.ArbitraryFeedForward, m_arbFeedforward / 12.);
    }

    @Override
    public void setEncoderPositionNU(double nu) {
        super.setSelectedSensorPosition(nu);
    }

    @Override
    public void setMotionProfileNU(double nu) {
        super.set(ControlMode.MotionMagic, nu, DemandType.ArbitraryFeedForward, m_arbFeedforward / 12.);
    }

    @Override
    public DataSignal<Double> getVelocityNU() {
        return new DataSignal<Double>(
            super::getSelectedSensorVelocity,
            (frequency) -> super.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, (int)(1000 / frequency)));
    }

    @Override
    public DataSignal<Double> getPositionNU(boolean latencyCompensated) {
        return new DataSignal<Double>(
            super::getSelectedSensorPosition,
            (frequency) -> super.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, (int)(1000 / frequency)));
    }

    @Override
    public DataSignal<Double> getOutputVoltage() {
        return new DataSignal<Double>(
            super::getMotorOutputVoltage,
            (frequency) -> super.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, (int)(1000 / frequency)));
    }

    @Override
    public BeakPIDConstants getPID() {
        SlotConfiguration config = new SlotConfiguration();
        super.getSlotConfigs(config, m_slot, 50);
        return new BeakPIDConstants(config);
    }

    public TalonSRXSimCollection getTalonSRXSimCollection() {
        return super.getTalonSRXSimCollection();
    }

    @Override
    public void set(double percentOutput) {
        super.set(ControlMode.PercentOutput, percentOutput, DemandType.ArbitraryFeedForward, m_arbFeedforward / 12.);
    }

    @Override
    public void setPID(BeakPIDConstants constants) {
        super.config_kP(m_slot, constants.kP);
        super.config_kI(m_slot, constants.kI);
        super.config_kD(m_slot, constants.kD);
        super.config_kF(m_slot, constants.kV);
    }

    @Override
    public DataSignal<Double> getSuppliedVoltage() {
        return new DataSignal<Double>(
            super::getBusVoltage,
            (frequency) -> super.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, (int)(1000 / frequency)));
    }

    @Override
    public void setVelocityConversionConstant(double constant) {
        m_velocityConversionConstant = constant;
    }

    @Override
    public double getVelocityConversionConstant() {
        return m_velocityConversionConstant;
    }

    @Override
    public void setPositionConversionConstant(double constant) {
        m_positionConversionConstant = constant;
    }

    @Override
    public double getPositionConversionConstant() {
        return m_positionConversionConstant;
    }

    @Override
    public void setEncoderGearRatio(double ratio) {
        m_gearRatio = ratio;
    }

    @Override
    public double getEncoderGearRatio() {
        return m_gearRatio;
    }

    @Override
    public void setWheelDiameter(Measure<Distance> diameter) {
        m_wheelDiameter = diameter;
    }

    @Override
    public Measure<Distance> getWheelDiameter() {
        return m_wheelDiameter;
    }

    @Override
    public void setNextArbFeedforward(double arbFeedforward) {
        m_arbFeedforward = arbFeedforward;
    }

    @Override
    public void useFOC(boolean useFoc) {
        return;
    }

    @Override
    public boolean getForwardLimitSwitch() {
        return m_forwardSource == BeakLimitSwitchSource.Connected ? super.isFwdLimitSwitchClosed() == 1
                : m_dioFwdLimitSwitch.get();
    }

    @Override
    public boolean getReverseLimitSwitch() {
        return m_reverseSource == BeakLimitSwitchSource.Connected ? super.isRevLimitSwitchClosed() == 1
                : m_dioRevLimitSwitch.get();
    }

    @Override
    public void applyConfig(BeakClosedLoopConfigs config) {
        FeedbackDevice device;
        switch (config.FeedbackSource) {
            case RemoteCANCoder:
                device = FeedbackDevice.RemoteSensor0;
                super.configRemoteFeedbackFilter(config.RemoteSensorID, RemoteSensorSource.CANCoder, 0);
                break;
            case ConnectedAbsolute:
                device = FeedbackDevice.CTRE_MagEncoder_Absolute;
                break;
            case ConnectedRelative:
                device = FeedbackDevice.CTRE_MagEncoder_Relative;
                break;
            case BuiltIn:
            default:
                device = FeedbackDevice.IntegratedSensor;
                break;
        }

        super.configSelectedFeedbackSensor(device);
    }

    @Override
    public void applyConfig(BeakCurrentLimitConfigs config) {
        super.configPeakCurrentLimit((int) config.SupplyCurrentThreshold);
        super.configPeakCurrentDuration((int) config.SupplyTimeThreshold * 1000);
        super.configContinuousCurrentLimit((int) config.SupplyCurrentLimit);
    }

    @Override
    public void applyConfig(BeakDutyCycleConfigs config) {
        super.configNeutralDeadband(config.NeutralDeadband);
        super.configOpenloopRamp(config.OpenRampPeriod);
        super.configClosedloopRamp(config.ClosedRampPeriod);
        super.configPeakOutputForward(config.PeakForwardOutput);
        super.configPeakOutputReverse(config.PeakReverseOutput);
    }

    @Override
    public void applyConfig(BeakHardwareLimitSwitchConfigs config) {
        switch (config.ForwardSource) {
            case DIO:
                if (m_dioFwdLimitSwitch != null) {
                    m_dioFwdLimitSwitch.close();
                }

                m_dioFwdLimitSwitch = new DigitalInput(config.ForwardLimitSwitchID);
                m_forwardSource = BeakLimitSwitchSource.DIO;
                break;
            case Connected:
                m_forwardSource = BeakLimitSwitchSource.Connected;
                super.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                        config.ForwardNormallyClosed ? LimitSwitchNormal.NormallyClosed
                                : LimitSwitchNormal.NormallyOpen);
                break;
            case None:
            default:
                break;
        }

        switch (config.ReverseSource) {
            case DIO:
                if (m_dioRevLimitSwitch != null) {
                    m_dioRevLimitSwitch.close();
                }

                m_dioRevLimitSwitch = new DigitalInput(config.ReverseLimitSwitchID);
                m_reverseSource = BeakLimitSwitchSource.DIO;
                break;
            case Connected:
                m_reverseSource = BeakLimitSwitchSource.Connected;
                super.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                        config.ReverseNormallyClosed ? LimitSwitchNormal.NormallyClosed
                                : LimitSwitchNormal.NormallyOpen);
                break;
            case None:
            default:
                break;
        }
    }

    @Override
    public void applyConfig(BeakMotionProfileConfigs config) {
        super.configMotionAcceleration(config.Acceleration.in(RPM.per(Second)) * getVelocityConversionConstant());
        super.configMotionCruiseVelocity(config.Velocity.in(RPM) * getVelocityConversionConstant());
    }

    @Override
    public void applyConfig(BeakVoltageConfigs config) {
        super.configNeutralDeadband(config.NeutralDeadband);
        super.configOpenloopRamp(config.OpenRampPeriod);
        super.configClosedloopRamp(config.ClosedRampPeriod);
        super.configPeakOutputForward(config.PeakForwardOutput / 12.0);
        super.configPeakOutputReverse(config.PeakReverseOutput / 12.0);
    }
}
