// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.motor;

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
import frc.lib.beaklib.motor.configs.BeakClosedLoopConfigs;
import frc.lib.beaklib.motor.configs.BeakCurrentLimitConfigs;
import frc.lib.beaklib.motor.configs.BeakDutyCycleConfigs;
import frc.lib.beaklib.motor.configs.BeakHardwareLimitSwitchConfigs;
import frc.lib.beaklib.motor.configs.BeakMotionProfileConfigs;
import frc.lib.beaklib.motor.configs.BeakVoltageConfigs;
import frc.lib.beaklib.motor.configs.BeakClosedLoopConfigs.FeedbackSensor;
import frc.lib.beaklib.motor.configs.BeakHardwareLimitSwitchConfigs.BeakLimitSwitchSource;
import frc.lib.beaklib.motor.requests.BeakControlRequest.OutputType;
import frc.lib.beaklib.pid.BeakPIDConstants;

// TODO: implement fake kS
/** Common motor controller interface for REV Spark MAX. */
public class BeakSparkMAX extends CANSparkMax implements BeakMotorController {
    private RelativeEncoder m_relativeEncoder;
    private AbsoluteEncoder m_absoluteEncoder;

    private SparkPIDController m_pid;

    private SparkLimitSwitch m_builtinRevLimitSwitch;
    private SparkLimitSwitch m_builtinFwdLimitSwitch;

    private DigitalInput m_dioRevLimitSwitch = null;
    private DigitalInput m_dioFwdLimitSwitch = null;

    private BeakLimitSwitchSource m_forwardSource = BeakLimitSwitchSource.None;
    private BeakLimitSwitchSource m_reverseSource = BeakLimitSwitchSource.None;

    private double m_velocityConversionConstant = 1.;
    private double m_positionConversionConstant = 1.;
    private double m_gearRatio = 1.;
    private Measure<Distance> m_wheelDiameter = Inches.of(4.);

    private int m_slot = 0;
    private double m_arbFeedforward = 0.;
    private double m_nominalVoltage;

    public BeakSparkMAX(int port) {
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
    public void setPID(BeakPIDConstants constants) {
        m_pid.setP(constants.kP, m_slot);
        m_pid.setI(constants.kI, m_slot);
        m_pid.setD(constants.kD, m_slot);
        m_pid.setFF(constants.kV, m_slot);
    }

    @Override
    public BeakPIDConstants getPID() {
        return new BeakPIDConstants(
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
    public void applyConfig(BeakClosedLoopConfigs config) {
        m_pid.setPositionPIDWrappingEnabled(config.Wrap);
        m_pid.setFeedbackDevice(
                config.FeedbackSource == FeedbackSensor.ConnectedAbsolute ? m_absoluteEncoder : m_relativeEncoder);
    }

    @Override
    public void applyConfig(BeakCurrentLimitConfigs config) {
        setSmartCurrentLimit((int) config.SupplyCurrentLimit);
    }

    @Override
    public void applyConfig(BeakDutyCycleConfigs config) {
        m_pid.setOutputRange(config.PeakReverseOutput, config.PeakForwardOutput);
        super.setClosedLoopRampRate(config.ClosedRampPeriod);
        super.setOpenLoopRampRate(config.OpenRampPeriod);
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
                m_builtinFwdLimitSwitch = super.getForwardLimitSwitch(
                        config.ForwardNormallyClosed ? Type.kNormallyClosed : Type.kNormallyOpen);

                m_forwardSource = BeakLimitSwitchSource.Connected;
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
                m_builtinRevLimitSwitch = super.getReverseLimitSwitch(
                        config.ReverseNormallyClosed ? Type.kNormallyClosed : Type.kNormallyOpen);
                m_reverseSource = BeakLimitSwitchSource.Connected;
                break;
            case None:
            default:
                break;
        }
    }

    @Override
    public void applyConfig(BeakMotionProfileConfigs config) {
        m_pid.setSmartMotionMaxAccel(config.Acceleration.in(RPM.per(Second)), m_slot);
        m_pid.setSmartMotionMaxVelocity(config.Velocity.in(RPM), m_slot);
    }

    @Override
    public void applyConfig(BeakVoltageConfigs config) {
        m_pid.setOutputRange(config.PeakReverseOutput / 12.0, config.PeakForwardOutput / 12.0);
        super.setClosedLoopRampRate(config.ClosedRampPeriod);
        super.setOpenLoopRampRate(config.OpenRampPeriod);
    }

    @Override
    public boolean getForwardLimitSwitch() {
        return m_forwardSource == BeakLimitSwitchSource.Connected ? m_builtinFwdLimitSwitch.isPressed()
                : m_dioFwdLimitSwitch.get();
    }

    @Override
    public boolean getReverseLimitSwitch() {
        return m_reverseSource == BeakLimitSwitchSource.Connected ? m_builtinRevLimitSwitch.isPressed()
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
}
