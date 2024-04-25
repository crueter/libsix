// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.motor;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import frc.lib.beaklib.motor.configs.BeakClosedLoopConfigs;
import frc.lib.beaklib.motor.configs.BeakCurrentLimitConfigs;
import frc.lib.beaklib.motor.configs.BeakDutyCycleConfigs;
import frc.lib.beaklib.motor.configs.BeakHardwareLimitSwitchConfigs;
import frc.lib.beaklib.motor.configs.BeakMotionProfileConfigs;
import frc.lib.beaklib.motor.configs.BeakVoltageConfigs;
import frc.lib.beaklib.pid.BeakPIDConstants;

/** A combined group of motor controllers to be controlled as one. */
public class BeakMotorControllerGroup implements BeakMotorController {
    private BeakMotorController[] m_controllers;

    private boolean m_inverted = false;

    public BeakMotorControllerGroup(BeakMotorController... motorControllers) {
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
        // reasons.
        // Sometimes, a user will pass in two motor controllers, one is inverted, one
        // isn't.
        // Say that, at some point, they need to invert this. It makes less sense to
        // change
        // the inversion of each and more sense to call a function that will invert each
        // motor
        // controller from its current state.

        m_inverted = isInverted;
        for (BeakMotorController controller : m_controllers) {
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
        for (BeakMotorController controller : m_controllers) {
            controller.setBrake(brake);
        }
    }

    @Override
    public void setVelocityNU(double nu) {
        for (BeakMotorController controller : m_controllers) {
            controller.setVelocityNU(nu);
        }
    }

    @Override
    public void setPositionNU(double nu) {
        for (BeakMotorController controller : m_controllers) {
            controller.setPositionNU(nu);
        }
    }

    @Override
    public void setEncoderPositionNU(double nu) {
        for (BeakMotorController controller : m_controllers) {
            controller.setEncoderPositionNU(nu);
        }
    }

    @Override
    public void setMotionProfileNU(double nu) {
        for (BeakMotorController controller : m_controllers) {
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
    public void setPID(BeakPIDConstants constants) {
        for (BeakMotorController controller : m_controllers) {
            controller.setPID(constants);
        }
    }

    @Override
    public BeakPIDConstants getPID() {
        return m_controllers[0].getPID();
    }

    @Override
    public void set(double percentOutput) {
        for (BeakMotorController controller : m_controllers) {
            controller.set(percentOutput);
        }
    }

    @Override
    public void setVelocityConversionConstant(double constant) {
        for (BeakMotorController controller : m_controllers) {
            controller.setVelocityConversionConstant(constant);
        }
    }

    @Override
    public double getVelocityConversionConstant() {
        return m_controllers[0].getVelocityConversionConstant();
    }

    @Override
    public void setPositionConversionConstant(double constant) {
        for (BeakMotorController controller : m_controllers) {
            controller.setPositionConversionConstant(constant);
        }
    }

    @Override
    public double getPositionConversionConstant() {
        return m_controllers[0].getPositionConversionConstant();
    }

    @Override
    public void setEncoderGearRatio(double ratio) {
        for (BeakMotorController controller : m_controllers) {
            controller.setEncoderGearRatio(ratio);
        }
    }

    @Override
    public double getEncoderGearRatio() {
        return m_controllers[0].getEncoderGearRatio();
    }

    @Override
    public void setWheelDiameter(Measure<Distance> diameter) {
        for (BeakMotorController controller : m_controllers) {
            controller.setWheelDiameter(diameter);
        }
    }

    @Override
    public Measure<Distance> getWheelDiameter() {
        return m_controllers[0].getWheelDiameter();
    }

    @Override
    public void setNextArbFeedforward(double arbFeedforward) {
        for (BeakMotorController controller : m_controllers) {
            controller.setNextArbFeedforward(arbFeedforward);
        }
    }

    @Override
    public void setSlot(int slot) {
        for (BeakMotorController controller : m_controllers) {
            controller.setSlot(slot);
        }
    }

    @Override
    public void useFOC(boolean useFoc) {
        for (BeakMotorController controller : m_controllers) {
            controller.useFOC(useFoc);
        }
    }

    @Override
    public void applyConfig(BeakClosedLoopConfigs config) {
        for (BeakMotorController controller : m_controllers) {
            controller.applyConfig(config);
        }
    }

    @Override
    public void applyConfig(BeakCurrentLimitConfigs config) {
        for (BeakMotorController controller : m_controllers) {
            controller.applyConfig(config);
        }
    }

    @Override
    public void applyConfig(BeakDutyCycleConfigs config) {
        for (BeakMotorController controller : m_controllers) {
            controller.applyConfig(config);
        }
    }

    @Override
    public void applyConfig(BeakHardwareLimitSwitchConfigs config) {
        for (BeakMotorController controller : m_controllers) {
            controller.applyConfig(config);
        }
    }

    @Override
    public void applyConfig(BeakMotionProfileConfigs config) {
        for (BeakMotorController controller : m_controllers) {
            controller.applyConfig(config);
        }
    }

    @Override
    public void applyConfig(BeakVoltageConfigs config) {
        for (BeakMotorController controller : m_controllers) {
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
}
