// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.motor;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.beaklib.CTRESignalStore;

/** Specifies methods to grab a value and its timestamp. */
public class DataSignal<T> {
    private final Supplier<T> m_value;
    private final Supplier<Double> m_timestamp;
    private final Runnable m_refresh;
    private final Consumer<Double> m_setUpdateFrequency;

    public DataSignal(Supplier<T> value, Consumer<Double> setUpdateFrequency) {
        this.m_value = value;
        this.m_setUpdateFrequency = setUpdateFrequency;

        this.m_refresh = () -> {};
        this.m_timestamp = () -> Timer.getFPGATimestamp();
    }

    public DataSignal(Supplier<T> value) {
        this(value, (freq) -> {});
    }

    public DataSignal(Supplier<T> value, Supplier<Double> timestamp, Runnable refresh,
            Consumer<Double> setUpdateFrequency) {
        this.m_value = value;
        this.m_timestamp = timestamp;
        this.m_refresh = refresh;
        this.m_setUpdateFrequency = setUpdateFrequency;
    }

    public DataSignal(StatusSignal<T> phoenixSignal) {
        CTRESignalStore.add(phoenixSignal);

        m_value = phoenixSignal::getValue;
        m_timestamp = () -> phoenixSignal.getTimestamp().getTime();
        m_refresh = phoenixSignal::refresh;
        m_setUpdateFrequency = phoenixSignal::setUpdateFrequency;
    }

    public T getValue() {
        return m_value.get();
    }

    public double getTimestamp() {
        return m_timestamp.get();
    }

    public void refresh() {
        m_refresh.run();
    }

    public void setUpdateFrequency(double frequencyHz) {
        m_setUpdateFrequency.accept(frequencyHz);
    }
}
