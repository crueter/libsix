// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib;

import java.util.ArrayList;

import com.ctre.phoenix6.BaseStatusSignal;

/** Signal store that updates CTRE signals periodically. */
public final class CTRESignalStore {
    private static ArrayList<BaseStatusSignal> signals = new ArrayList<>();

    public static void add(BaseStatusSignal... signal) {
        for (BaseStatusSignal status : signal) {
            signals.add(status);
            System.out.println(signals);
        }
    }

    public static void update() {
        BaseStatusSignal.refreshAll(signals.toArray(new BaseStatusSignal[0]));
    }
}
