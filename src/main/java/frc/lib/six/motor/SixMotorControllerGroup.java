// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.six.motor;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import frc.lib.six.motor.configs.SixClosedLoopConfigs;
import frc.lib.six.motor.configs.SixCurrentConfigs;
import frc.lib.six.motor.configs.SixCurrentLimitConfigs;
import frc.lib.six.motor.configs.SixDutyCycleConfigs;
import frc.lib.six.motor.configs.SixHardwareLimitSwitchConfigs;
import frc.lib.six.motor.configs.SixMotionProfileConfigs;
import frc.lib.six.motor.configs.SixSoftLimitConfigs;
import frc.lib.six.motor.configs.SixVoltageConfigs;
import frc.lib.six.motor.requests.SixControlRequest.OutputType;
import frc.lib.six.pid.SixPIDConstants;

/** A combined group of motor controllers to be controlled as one. */
public class SixMotorControllerGroup implements SixMotorController {
    private SixMotorController[] m_controllers;

    private boolean m_inverted = false;

    public SixMotorControllerGroup(SixMotorController... motorControllers) {
        m_controllers = motorControllers;
    }

    @Override
    public double get() {
        // all should have the same applied output
        return m_controllers[0].get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        // This is an interesting implementation but one I'm keeping for various
        // reasons. Sometimes, a user will pass in two motor controllers, one is
        // inverted, one isn't. Say that, at some point, they need to invert this. It
        // makes less sense to change the inversion of each and more sense to call a
        // function that will invert each motor controller from its current state.

        m_inverted = isInverted;
        for (SixMotorController controller : m_controllers) {
            // :)
            controller.setInverted(isInverted ^ controller.getInverted());
        }
    }

    @Override
    public boolean getInverted() {
        return m_inverted;
    }

    @Override
    public void setBrake(boolean brake) {
        for (SixMotorController controller : m_controllers) {
            controller.setBrake(brake);
        }
    }

    @Override
    public void setVelocityNU(double nu) {
        for (SixMotorController controller : m_controllers) {
            controller.setVelocityNU(nu);
        }
    }

    @Override
    public void setPositionNU(double nu) {
        for (SixMotorController controller : m_controllers) {
            controller.setPositionNU(nu);
        }
    }

    @Override
    public void setEncoderPositionNU(double nu) {
        for (SixMotorController controller : m_controllers) {
            controller.setEncoderPositionNU(nu);
        }
    }

    @Override
    public void setMotionProfileNU(double nu) {
        for (SixMotorController controller : m_controllers) {
            controller.setMotionProfileNU(nu);
        }
    }

    @Override
    public DataSignal<Double> getVelocityNU() {
        return m_controllers[0].getVelocityNU();
    }

    @Override
    public DataSignal<Double> getPositionNU(boolean latencyCompensated) {
        return m_controllers[0].getPositionNU(latencyCompensated);
    }

    @Override
    public DataSignal<Double> getSuppliedVoltage() {
        return m_controllers[0].getSuppliedVoltage();
    }

    @Override
    public void setPID(SixPIDConstants constants) {
        for (SixMotorController controller : m_controllers) {
            controller.setPID(constants);
        }
    }

    @Override
    public SixPIDConstants getPID() {
        return m_controllers[0].getPID();
    }

    @Override
    public void set(double percentOutput) {
        for (SixMotorController controller : m_controllers) {
            controller.set(percentOutput);
        }
    }

    @Override
    public void setVelocityConversionConstant(double constant) {
        for (SixMotorController controller : m_controllers) {
            controller.setVelocityConversionConstant(constant);
        }
    }

    @Override
    public double getVelocityConversionConstant() {
        return m_controllers[0].getVelocityConversionConstant();
    }

    @Override
    public void setPositionConversionConstant(double constant) {
        for (SixMotorController controller : m_controllers) {
            controller.setPositionConversionConstant(constant);
        }
    }

    @Override
    public double getPositionConversionConstant() {
        return m_controllers[0].getPositionConversionConstant();
    }

    @Override
    public void setEncoderGearRatio(double ratio) {
        for (SixMotorController controller : m_controllers) {
            controller.setEncoderGearRatio(ratio);
        }
    }

    @Override
    public double getEncoderGearRatio() {
        return m_controllers[0].getEncoderGearRatio();
    }

    @Override
    public void setWheelDiameter(Measure<Distance> diameter) {
        for (SixMotorController controller : m_controllers) {
            controller.setWheelDiameter(diameter);
        }
    }

    @Override
    public Measure<Distance> getWheelDiameter() {
        return m_controllers[0].getWheelDiameter();
    }

    @Override
    public void setNextArbFeedforward(double arbFeedforward) {
        for (SixMotorController controller : m_controllers) {
            controller.setNextArbFeedforward(arbFeedforward);
        }
    }

    @Override
    public void setSlot(int slot) {
        for (SixMotorController controller : m_controllers) {
            controller.setSlot(slot);
        }
    }

    @Override
    public void useFOC(boolean useFoc) {
        for (SixMotorController controller : m_controllers) {
            controller.useFOC(useFoc);
        }
    }

    @Override
    public void applyConfig(SixClosedLoopConfigs config) {
        for (SixMotorController controller : m_controllers) {
            controller.applyConfig(config);
        }
    }

    @Override
    public void applyConfig(SixCurrentLimitConfigs config) {
        for (SixMotorController controller : m_controllers) {
            controller.applyConfig(config);
        }
    }

    @Override
    public void applyConfig(SixDutyCycleConfigs config) {
        for (SixMotorController controller : m_controllers) {
            controller.applyConfig(config);
        }
    }

    @Override
    public void applyConfig(SixHardwareLimitSwitchConfigs config) {
        for (SixMotorController controller : m_controllers) {
            controller.applyConfig(config);
        }
    }

    @Override
    public void applyConfig(SixMotionProfileConfigs config) {
        for (SixMotorController controller : m_controllers) {
            controller.applyConfig(config);
        }
    }

    @Override
    public void applyConfig(SixVoltageConfigs config) {
        for (SixMotorController controller : m_controllers) {
            controller.applyConfig(config);
        }
    }

    @Override
    public boolean getForwardLimitSwitch() {
        return m_controllers[0].getForwardLimitSwitch();
    }

    @Override
    public boolean getReverseLimitSwitch() {
        return m_controllers[0].getReverseLimitSwitch();
    }

    @Override
    public void setNextOutputType(OutputType outputType) {
        for (SixMotorController controller : m_controllers) {
            controller.setNextOutputType(outputType);
        }
    }

    @Override
    public void setNominalVoltage(double volts) {
        for (SixMotorController controller : m_controllers) {
            controller.setNominalVoltage(volts);
        }
    }

    @Override
    public void setCurrent(double amps) {
        for (SixMotorController controller : m_controllers) {
            controller.setCurrent(amps);
        }
    }

    @Override
    public void applyConfig(SixSoftLimitConfigs config) {
        for (SixMotorController controller : m_controllers) {
            controller.applyConfig(config);
        }
    }

    @Override
    public void applyConfig(SixCurrentConfigs config) {
        for (SixMotorController controller : m_controllers) {
            controller.applyConfig(config);
        }
    }
}
