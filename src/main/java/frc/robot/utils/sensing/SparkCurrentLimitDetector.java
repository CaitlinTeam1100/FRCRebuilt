// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.sensing;

import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

/** Add your docs here. */
public class SparkCurrentLimitDetector {
    static private final double c_defaultDebounceTime = 0.1;

    public enum HardLimitDirection {
        kForward, //Hit forward limit
        kFree, //Limit is not engaged
        kReverse //Reverse limit hit
    }

    private final double m_velocityTolerance;
    private SparkBase m_motorController;
    private double m_tripPoint;
    private Debouncer m_forwardDebouncer;
    private Debouncer m_reverseDebouncer;
    private HardLimitDirection m_tripDirection;

    public SparkCurrentLimitDetector(SparkBase motorController, double currentTripPoint, double zeroSpeedTolerance) {
        this(motorController, currentTripPoint, zeroSpeedTolerance, c_defaultDebounceTime);
    }

    public SparkCurrentLimitDetector(SparkBase motorController, double currentTripPoint, double zeroSpeedTolerance, double debounceTime) {
        m_motorController = motorController;
        m_tripPoint = currentTripPoint;
        m_velocityTolerance = zeroSpeedTolerance;
        m_forwardDebouncer = new Debouncer(debounceTime, DebounceType.kRising);
        m_reverseDebouncer = new Debouncer(debounceTime, DebounceType.kRising);
        m_tripDirection = HardLimitDirection.kFree;
    }

    public HardLimitDirection check() {
        double motorVelocity = m_motorController.getEncoder().getVelocity();
        boolean forwardLimitHit = m_forwardDebouncer.calculate(
            (m_motorController.getOutputCurrent() > m_tripPoint) &&
            (MathUtil.isNear(0, motorVelocity, m_velocityTolerance) &&
            (m_motorController.getAppliedOutput() > 0))
        );

        boolean reverseLimitHit = m_reverseDebouncer.calculate(
            (m_motorController.getOutputCurrent() > m_tripPoint) &&
            (MathUtil.isNear(0, motorVelocity, m_velocityTolerance)) &&
            (m_motorController.getAppliedOutput() < 0));

        if(forwardLimitHit && reverseLimitHit) {
            System.out.println("ERROR, both forward and reverse limits triggered");
        }

        if(forwardLimitHit) {
            m_tripDirection = HardLimitDirection.kForward;
        }
        else if (reverseLimitHit)
        {
            m_tripDirection = HardLimitDirection.kReverse;
        }
        else if(!MathUtil.isNear(0, motorVelocity, m_velocityTolerance)){
            m_tripDirection = HardLimitDirection.kFree;
        }

        return m_tripDirection;
    }
}
