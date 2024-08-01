// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.six.motor;

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
import frc.lib.six.motor.configs.SixClosedLoopConfigs;
import frc.lib.six.motor.configs.SixCurrentConfigs;
import frc.lib.six.motor.configs.SixCurrentLimitConfigs;
import frc.lib.six.motor.configs.SixDutyCycleConfigs;
import frc.lib.six.motor.configs.SixHardwareLimitSwitchConfigs;
import frc.lib.six.motor.configs.SixMotionProfileConfigs;
import frc.lib.six.motor.configs.SixSoftLimitConfigs;
import frc.lib.six.motor.configs.SixVoltageConfigs;
import frc.lib.six.motor.configs.SixHardwareLimitSwitchConfigs.SixLimitSwitchSource;
import frc.lib.six.motor.requests.SixControlRequest.OutputType;
import frc.lib.six.pid.SixPIDConstants;

/** Common motor controller interface for Talon SRX. */
public class SixTalonSRX extends WPI_TalonSRX implements SixMotorController {
    private double m_velocityConversionConstant = 4096. / 600.;
    private double m_positionConversionConstant = 4096.;
    private double m_gearRatio = 1.;
    private Measure<Distance> m_wheelDiameter = Inches.of(4.);

    private int m_slot = 0;
    private double m_arbFeedforward = 0.;
    private double m_nominalVoltage;

    private DigitalInput m_dioRevLimitSwitch = null;
    private DigitalInput m_dioFwdLimitSwitch = null;

    private SixLimitSwitchSource m_forwardSource = SixLimitSwitchSource.None;
    private SixLimitSwitchSource m_reverseSource = SixLimitSwitchSource.None;

    public SixTalonSRX(int port) {
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
                (frequency) -> super.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, (int) (1000 / frequency)));
    }

    @Override
    public DataSignal<Double> getPositionNU(boolean latencyCompensated) {
        return new DataSignal<Double>(
                super::getSelectedSensorPosition,
                (frequency) -> super.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, (int) (1000 / frequency)));
    }

    @Override
    public DataSignal<Double> getOutputVoltage() {
        return new DataSignal<Double>(
                super::getMotorOutputVoltage,
                (frequency) -> super.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, (int) (1000 / frequency)));
    }

    @Override
    public SixPIDConstants getPID() {
        SlotConfiguration config = new SlotConfiguration();
        super.getSlotConfigs(config, m_slot, 50);
        return new SixPIDConstants(config);
    }

    public TalonSRXSimCollection getTalonSRXSimCollection() {
        return super.getTalonSRXSimCollection();
    }

    @Override
    public void set(double percentOutput) {
        super.set(ControlMode.PercentOutput, percentOutput, DemandType.ArbitraryFeedForward, m_arbFeedforward / 12.);
    }

    @Override
    public void setPID(SixPIDConstants constants) {
        super.config_kP(m_slot, constants.kP);
        super.config_kI(m_slot, constants.kI);
        super.config_kD(m_slot, constants.kD);
        super.config_kF(m_slot, constants.kV);
    }

    @Override
    public DataSignal<Double> getSuppliedVoltage() {
        return new DataSignal<Double>(
                super::getBusVoltage,
                (frequency) -> super.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, (int) (1000 / frequency)));
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
        return m_forwardSource == SixLimitSwitchSource.Connected ? super.isFwdLimitSwitchClosed() == 1
                : m_dioFwdLimitSwitch.get();
    }

    @Override
    public boolean getReverseLimitSwitch() {
        return m_reverseSource == SixLimitSwitchSource.Connected ? super.isRevLimitSwitchClosed() == 1
                : m_dioRevLimitSwitch.get();
    }

    @Override
    public void applyConfig(SixClosedLoopConfigs config) {
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
    public void applyConfig(SixCurrentLimitConfigs config) {
        super.configPeakCurrentLimit((int) config.SupplyCurrentThreshold);
        super.configPeakCurrentDuration((int) config.SupplyTimeThreshold * 1000);
        super.configContinuousCurrentLimit((int) config.SupplyCurrentLimit);
    }

    @Override
    public void applyConfig(SixDutyCycleConfigs config) {
        super.configNeutralDeadband(config.NeutralDeadband);
        super.configOpenloopRamp(config.OpenRampPeriod);
        super.configClosedloopRamp(config.ClosedRampPeriod);
        super.configPeakOutputForward(config.PeakForwardOutput);
        super.configPeakOutputReverse(config.PeakReverseOutput);
    }

    @Override
    public void applyConfig(SixHardwareLimitSwitchConfigs config) {
        switch (config.ForwardSource) {
            case DIO:
                if (m_dioFwdLimitSwitch != null) {
                    m_dioFwdLimitSwitch.close();
                }

                m_dioFwdLimitSwitch = new DigitalInput(config.ForwardLimitSwitchID);
                m_forwardSource = SixLimitSwitchSource.DIO;
                break;
            case Connected:
                m_forwardSource = SixLimitSwitchSource.Connected;
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
                m_reverseSource = SixLimitSwitchSource.DIO;
                break;
            case Connected:
                m_reverseSource = SixLimitSwitchSource.Connected;
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
    public void applyConfig(SixMotionProfileConfigs config) {
        super.configMotionAcceleration(config.Acceleration.in(RPM.per(Second)) * getVelocityConversionConstant());
        super.configMotionCruiseVelocity(config.Velocity.in(RPM) * getVelocityConversionConstant());
    }

    @Override
    public void applyConfig(SixVoltageConfigs config) {
        super.configNeutralDeadband(config.NeutralDeadband);
        super.configOpenloopRamp(config.OpenRampPeriod);
        super.configClosedloopRamp(config.ClosedRampPeriod);
        super.configPeakOutputForward(config.PeakForwardOutput / 12.0);
        super.configPeakOutputReverse(config.PeakReverseOutput / 12.0);
    }

    @Override
    public void setNextOutputType(OutputType outputType) {
        super.enableVoltageCompensation(outputType == OutputType.Voltage);
    }

    @Override
    public void setNominalVoltage(double volts) {
        m_nominalVoltage = volts;
        super.configVoltageCompSaturation(m_nominalVoltage);
    }

    @Override
    public void setCurrent(double amps) {
        super.set(ControlMode.Current, amps);
    }

    @Override
    public void applyConfig(SixSoftLimitConfigs config) {
        super.configForwardSoftLimitThreshold(config.ForwardLimit.getRotations() * getPositionConversionConstant() * getEncoderGearRatio());
        super.configForwardSoftLimitEnable(config.ForwardLimit.getRotations() != 0.0);
        super.configReverseSoftLimitThreshold(config.ReverseLimit.getRotations() * getPositionConversionConstant() * getEncoderGearRatio());
        super.configReverseSoftLimitEnable(config.ReverseLimit.getRotations() != 0.0);
    }

    @Override
    public void applyConfig(SixCurrentConfigs config) {
    }
}
