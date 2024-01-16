// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MMUtilities;

/** Add your docs here. */
public class MMRollingAvg {
    private int sampleCount;
    private double[] samples;
    private int ptr = 0;
    private double sum;

    public MMRollingAvg(int sampleCount) {
        this.sampleCount = sampleCount;
        samples = new double[sampleCount];
    }

    public double update(double currentValue) {
        sum -= samples[ptr];
        sum += currentValue;
        samples[ptr] = currentValue;
        ptr = (ptr + 1) % sampleCount;

        return sum / sampleCount;
    }
}
