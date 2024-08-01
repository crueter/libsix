// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.six.motor;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.six.motor.configs.SixClosedLoopConfigs;
import frc.lib.six.motor.configs.SixCurrentConfigs;
import frc.lib.six.motor.configs.SixCurrentLimitConfigs;
import frc.lib.six.motor.configs.SixDutyCycleConfigs;
import frc.lib.six.motor.configs.SixHardwareLimitSwitchConfigs;
import frc.lib.six.motor.configs.SixMotionProfileConfigs;
import frc.lib.six.motor.configs.SixSoftLimitConfigs;
import frc.lib.six.motor.configs.SixVoltageConfigs;
import frc.lib.six.motor.configs.SixClosedLoopConfigs.FeedbackSensor;
import frc.lib.six.motor.configs.SixHardwareLimitSwitchConfigs.SixLimitSwitchSource;
import frc.lib.six.motor.requests.SixControlRequest.OutputType;
import frc.lib.six.pid.SixPIDConstants;

// TODO: implement fake kS
/** Common motor controller interface for REV Spark MAX. */
public class SixSparkMAX extends CANSparkMax implements SixMotorController {
    private RelativeEncoder m_relativeEncoder;
    private AbsoluteEncoder m_absoluteEncoder;

    private SparkPIDController m_pid;

    private SparkLimitSwitch m_builtinRevLimitSwitch;
    private SparkLimitSwitch m_builtinFwdLimitSwitch;

    private DigitalInput m_dioRevLimitSwitch = null;
    private DigitalInput m_dioFwdLimitSwitch = null;

    private SixLimitSwitchSource m_forwardSource = SixLimitSwitchSource.None;
    private SixLimitSwitchSource m_reverseSource = SixLimitSwitchSource.None;

    private double m_velocityConversionConstant = 1.;
    private double m_positionConversionConstant = 1.;
    private double m_gearRatio = 1.;
    private Measure<Distance> m_wheelDiameter = Inches.of(4.);

    private int m_slot = 0;
    private double m_arbFeedforward = 0.;
    private double m_nominalVoltage;

    public SixSparkMAX(int port) {
        super(port, MotorType.kBrushless);

        resetControllers();
    }

    @Override
    public void setBrake(boolean brake) {
        super.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setVelocityNU(double nu) {
        m_pid.setReference(nu, ControlType.kVelocity, m_slot, m_arbFeedforward);
    }

    @Override
    public void setPositionNU(double nu) {
        SmartDashboard.putNumber("PID " + super.getDeviceId(), nu - m_relativeEncoder.getPosition());
        m_pid.setReference(nu, ControlType.kPosition, m_slot, m_arbFeedforward);
    }

    @Override
    public void setEncoderPositionNU(double nu) {
        m_relativeEncoder.setPosition(nu);
    }

    @Override
    public void setMotionProfileNU(double nu) {
        m_pid.setReference(nu, ControlType.kSmartMotion, m_slot, m_arbFeedforward);
    }

    @Override
    public DataSignal<Double> getVelocityNU() {
        return new DataSignal<Double>(
                m_relativeEncoder::getVelocity,
                (frequency) -> setPeriodicFramePeriod(PeriodicFrame.kStatus1, (int) (1000 / frequency)));
    }

    @Override
    public DataSignal<Double> getPositionNU(boolean latencyCompensated) {
        return new DataSignal<Double>(
                m_relativeEncoder::getPosition,
                (frequency) -> setPeriodicFramePeriod(PeriodicFrame.kStatus2, (int) (1000 / frequency)));
    }

    @Override
    public void set(double percentOutput) {
        m_pid.setReference(percentOutput, ControlType.kDutyCycle, 0, m_arbFeedforward);
    }

    @Override
    public void setPID(SixPIDConstants constants) {
        m_pid.setP(constants.kP, m_slot);
        m_pid.setI(constants.kI, m_slot);
        m_pid.setD(constants.kD, m_slot);
        m_pid.setFF(constants.kV, m_slot);
    }

    @Override
    public SixPIDConstants getPID() {
        return new SixPIDConstants(
                m_pid.getP(m_slot),
                m_pid.getI(m_slot),
                m_pid.getD(m_slot),
                m_pid.getFF(m_slot));
    }

    @Override
    public DataSignal<Double> getSuppliedVoltage() {
        return new DataSignal<Double>(
                super::getBusVoltage,
                (frequency) -> setPeriodicFramePeriod(PeriodicFrame.kStatus1, (int) (1000 / frequency)));

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

    private void resetControllers() {
        m_relativeEncoder = super.getEncoder();
        // m_alternateEncoder = super.getAlternateEncoder(kAPIBuildVersion);
        m_absoluteEncoder = super.getAbsoluteEncoder();
        m_pid = super.getPIDController();

        m_builtinRevLimitSwitch = super.getReverseLimitSwitch(Type.kNormallyOpen);
        m_builtinFwdLimitSwitch = super.getForwardLimitSwitch(Type.kNormallyOpen);
    }

    @Override
    public void setNextArbFeedforward(double arbFeedforward) {
        m_arbFeedforward = arbFeedforward;
    }

    @Override
    public void setSlot(int slot) {
        m_slot = slot;
    }

    @Override
    public void useFOC(boolean useFoc) {
        return;
    }

    @Override
    public void applyConfig(SixClosedLoopConfigs config) {
        m_pid.setPositionPIDWrappingEnabled(config.Wrap);
        m_pid.setFeedbackDevice(
                config.FeedbackSource == FeedbackSensor.ConnectedAbsolute ? m_absoluteEncoder : m_relativeEncoder);
    }

    @Override
    public void applyConfig(SixCurrentLimitConfigs config) {
        setSmartCurrentLimit((int) config.SupplyCurrentLimit);
    }

    @Override
    public void applyConfig(SixDutyCycleConfigs config) {
        m_pid.setOutputRange(config.PeakReverseOutput, config.PeakForwardOutput);
        super.setClosedLoopRampRate(config.ClosedRampPeriod);
        super.setOpenLoopRampRate(config.OpenRampPeriod);
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
                m_builtinFwdLimitSwitch = super.getForwardLimitSwitch(
                        config.ForwardNormallyClosed ? Type.kNormallyClosed : Type.kNormallyOpen);

                m_forwardSource = SixLimitSwitchSource.Connected;
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
                m_builtinRevLimitSwitch = super.getReverseLimitSwitch(
                        config.ReverseNormallyClosed ? Type.kNormallyClosed : Type.kNormallyOpen);
                m_reverseSource = SixLimitSwitchSource.Connected;
                break;
            case None:
            default:
                break;
        }
    }

    @Override
    public void applyConfig(SixMotionProfileConfigs config) {
        m_pid.setSmartMotionMaxAccel(config.Acceleration.in(RPM.per(Second)), m_slot);
        m_pid.setSmartMotionMaxVelocity(config.Velocity.in(RPM), m_slot);
    }

    @Override
    public void applyConfig(SixVoltageConfigs config) {
        m_pid.setOutputRange(config.PeakReverseOutput / 12.0, config.PeakForwardOutput / 12.0);
        super.setClosedLoopRampRate(config.ClosedRampPeriod);
        super.setOpenLoopRampRate(config.OpenRampPeriod);
    }

    @Override
    public boolean getForwardLimitSwitch() {
        return m_forwardSource == SixLimitSwitchSource.Connected ? m_builtinFwdLimitSwitch.isPressed()
                : m_dioFwdLimitSwitch.get();
    }

    @Override
    public boolean getReverseLimitSwitch() {
        return m_reverseSource == SixLimitSwitchSource.Connected ? m_builtinRevLimitSwitch.isPressed()
                : m_dioRevLimitSwitch.get();
    }

    @Override
    public void setNextOutputType(OutputType outputType) {
        switch (outputType) {
            case DutyCycle:
                super.disableVoltageCompensation();
                break;
            case Voltage:
                super.enableVoltageCompensation(m_nominalVoltage);
                break;
            case Current:
            default:
                break;
        }
    }

    @Override
    public void setNominalVoltage(double volts) {
        m_nominalVoltage = volts;
    }

    @Override
    public void setCurrent(double amps) {
        m_pid.setReference(amps, ControlType.kCurrent);
    }

    @Override
    public void applyConfig(SixSoftLimitConfigs config) {
        super.setSoftLimit(SoftLimitDirection.kForward,
                (float) (config.ForwardLimit.getRotations() * getPositionConversionConstant() * getEncoderGearRatio()));

        super.setSoftLimit(SoftLimitDirection.kReverse,
                (float) (config.ReverseLimit.getRotations() * getPositionConversionConstant() * getEncoderGearRatio()));

    }

    @Override
    public void applyConfig(SixCurrentConfigs config) {
    }
}
