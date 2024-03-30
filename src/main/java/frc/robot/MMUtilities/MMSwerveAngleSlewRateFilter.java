// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MMUtilities;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class MMSwerveAngleSlewRateFilter {
  private final Rotation2d slewRateLimit;
  private Rotation2d prevValue;
  private double prevTime;
  private double slewRateLimitDegrees;

  /**
   * Creates a new SlewRateLimiter with the given rate limit and initial
   * value.
   *
   * @param slewRateLimit The rate-of-change limit, in units per second.
   * @param initialValue  The initial value of the input.
   */
  public MMSwerveAngleSlewRateFilter(Rotation2d slewRateLimit, Rotation2d initialValue) {
    this.slewRateLimit = slewRateLimit;
    prevValue = initialValue;
    prevTime = MathSharedStore.getTimestamp();
    slewRateLimitDegrees = slewRateLimit.getDegrees();
  }

  /**
   * Creates a new SlewRateLimiter with the given rate limit.
   *
   * @param rateLimit The rate-of-change limit, in units per second.
   */
  public MMSwerveAngleSlewRateFilter(Rotation2d rateLimit) {
    this(rateLimit, new Rotation2d());
  }

  /**
   * Filters the input to limit its slew rate.
   *
   * @param input The input value whose slew rate is to be limited.
   * @return The filtered value, which will not change faster than the slew rate.
   */
  public Rotation2d calculate(Rotation2d input) {
    double currentTime = MathSharedStore.getTimestamp();
    double elapsedTime = currentTime - prevTime;

    // Flip previous to current to account for swerve drive optimization.
    if (Math.abs(prevValue.minus(input).getDegrees()) > 90) {
      prevValue = prevValue.rotateBy(Rotation2d.fromDegrees(180));
    }

    prevValue = prevValue
        .plus(
            Rotation2d.fromDegrees(
                MathUtil.clamp(
                    input.minus(prevValue).getDegrees(),
                    -slewRateLimitDegrees * elapsedTime,
                    slewRateLimitDegrees * elapsedTime)));

    prevTime = currentTime;
    return prevValue;
  }

  /**
   * Returns the value last calculated by the SlewRateLimiter.
   *
   * @return The last value.
   */
  public Rotation2d lastValue() {
    return prevValue;
  }

  /**
   * Resets the slew rate limiter to the specified value; ignores the rate limit
   * when doing so.
   *
   * @param value The value to reset to.
   */
  public void reset(Rotation2d value) {
    prevValue = value;
    prevTime = MathSharedStore.getTimestamp();
  }

}
