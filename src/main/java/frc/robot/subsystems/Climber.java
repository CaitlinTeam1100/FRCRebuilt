// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;

import java.math.MathContext;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.utils.sensing.SparkCurrentLimitDetector;
import frc.robot.utils.sensing.SparkCurrentLimitDetector.HardLimitDirection;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private static Climber m_Climber;

  private boolean m_climberEnabled;

  private double m_maxInches = 8;

  private SparkBase m_climberMotor;

  private SparkBaseConfig m_climberConfig;

  private SparkCurrentLimitDetector m_climberCurrentLimit;

  private boolean m_tuneClimber;

  private TDNumber m_TDclimberP;
  private TDNumber m_TDclimberI;
  private TDNumber m_TDclimberD;
  private double m_climberP;
  private double m_climberI;
  private double m_climberD;

  private TDNumber m_TDclimberKs;
  private TDNumber m_TDclimberKg;
  private TDNumber m_TDclimberKv;
  private TDNumber m_TDclimberKa;

  private SparkCurrentLimitDetector m_climberLimiter;

  private TDNumber m_TDclimberTargetPosition;
  private TDNumber m_TDclimberMeasuredPosition;
  private TDNumber m_TDclimberProfilePosition;
  private TDNumber m_TDclimberCurrentOutput;

  private ElevatorFeedforward m_climberFeedForwardController;
  private TrapezoidProfile m_climberProfile;
  private TrapezoidProfile.State m_climberState;
  private TrapezoidProfile.State m_climberSetpoint;

  private Climber() {
    super("Climber");
    m_climberEnabled = cfgBool("climberEnabled");

    if (m_climberEnabled) {
      var climberMotorConfig = config().getMotorController("climber");
      m_climberMotor = climberMotorConfig.m_controller;

      m_tuneClimber = cfgBool("tuneClimber");

      m_TDclimberP = new TDNumber(this, "Tuning", "climbP");
      m_TDclimberI = new TDNumber(this, "Tuning", "climbI");
      m_TDclimberD = new TDNumber(this, "Tuning", "climbD");
      m_TDclimberP.set(cfgDbl("climberP"));
      m_TDclimberI.set(cfgDbl("climberI"));
      m_TDclimberD.set(cfgDbl("climberD"));
      m_climberP = m_TDclimberP.get();
      m_climberI = m_TDclimberI.get();
      m_climberD = m_TDclimberD.get();

      m_TDclimberKs = new TDNumber(this, "Tuning", "climbkS");
      m_TDclimberKg = new TDNumber(this, "Tuning", "climbkG");
      m_TDclimberKv = new TDNumber(this, "Tuning", "climbkV");
      m_TDclimberKa = new TDNumber(this, "Tuning", "climbkA");
      m_TDclimberKs.set(cfgDbl("climberKs"));
      m_TDclimberKg.set(cfgDbl("climberKg"));
      m_TDclimberKv.set(cfgDbl("climberKv"));
      m_TDclimberKa.set(cfgDbl("climberKa"));

      m_climberConfig = climberMotorConfig.m_config;
      m_climberConfig.inverted(true);
      m_climberConfig.closedLoop.pid(m_climberP, m_climberI, m_climberD);
      m_climberConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
      m_climberConfig.closedLoop.positionWrappingEnabled(false);
      m_climberConfig.encoder.positionConversionFactor(cfgDbl("climberRatio"));
      m_climberMotor.configure(m_climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      m_climberCurrentLimit = new SparkCurrentLimitDetector(m_climberMotor, cfgDbl("climberCurrentLimit"), cfgDbl("climberZeroVelocityTolerance"));

      m_climberFeedForwardController = new ElevatorFeedforward(
          m_TDclimberKs.get(),
          m_TDclimberKg.get(),
          m_TDclimberKv.get(),
          m_TDclimberKa.get());

      m_climberProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
          cfgDbl("climberMaxVelocity"), cfgDbl("climberMaxAcceleration")));

      m_climberSetpoint = new TrapezoidProfile.State(m_climberMotor.getEncoder().getPosition(), 0.0);
      m_climberState = new TrapezoidProfile.State(m_climberMotor.getEncoder().getPosition(), 0.0);

      m_TDclimberTargetPosition = new TDNumber(this, "Position", "Target Position");

      m_TDclimberProfilePosition = new TDNumber(this, "Position", "Profile Position");
      m_TDclimberProfilePosition.set(m_climberState.position);

      m_TDclimberMeasuredPosition = new TDNumber(this, "Position", "Measured Position");

      m_TDclimberCurrentOutput = new TDNumber(this, "Current", "Measured Output");
    }
  }

  public static Climber getInstance() {
    if (m_Climber == null) {
      m_Climber = new Climber();
    }
    return m_Climber;
  }

  public void FunnyClimbAction() {
  }

  public void FunnyClimbAscendAction() {
  }

  public void FunnyClimbDescendAction() {
  }

  public void FunnyClimbHaltAction() {
  }

  public double getClimberTargetAngle() {
    return m_TDclimberTargetPosition.get();
  }

  public double getClimberAngle() {
    return m_climberMotor.getEncoder().getPosition();
  }

  public boolean isClimberOnRung() {
    return false;
  }

  public void setClimberTargetAngle(double positionInches) {
    m_TDclimberTargetPosition.set(positionInches);
  }

  public double clampTargetAngle(double positionInches) {
    return MathUtil.clamp(positionInches, 0, m_maxInches);
  }

  @Override
  public void periodic() {
    if (!m_climberEnabled)
      return;

    if (m_tuneClimber) {
      if (m_TDclimberP.get() != m_climberP ||
          m_TDclimberI.get() != m_climberI ||
          m_TDclimberD.get() != m_climberD) {
        m_climberP = m_TDclimberP.get();
        m_climberI = m_TDclimberI.get();
        m_climberD = m_TDclimberD.get();
        m_climberConfig.closedLoop.pid(m_climberP, m_climberI, m_climberD);
        m_climberMotor.configure(m_climberConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
      }
      m_climberFeedForwardController.setKg(m_TDclimberKg.get());
      m_climberFeedForwardController.setKs(m_TDclimberKs.get());
      m_climberFeedForwardController.setKv(m_TDclimberKv.get());
      m_climberFeedForwardController.setKa(m_TDclimberKa.get());
    }

    HardLimitDirection hardLimit = m_climberCurrentLimit.check();
    if (hardLimit == HardLimitDirection.kForward && m_climberMotor.getEncoder().getVelocity() > 0) {
      m_climberMotor.getEncoder().setPosition(m_maxInches);
      m_TDclimberTargetPosition.set(m_climberMotor.getEncoder().getPosition());
    } else if (hardLimit == HardLimitDirection.kReverse && m_climberMotor.getEncoder().getVelocity() < 0) {
      m_climberMotor.getEncoder().setPosition(0);
      m_TDclimberTargetPosition.set(m_climberMotor.getEncoder().getPosition());
    }
 
    m_climberSetpoint = new TrapezoidProfile.State(clampTargetAngle(m_TDclimberTargetPosition.get()), 0.0);
    m_climberState = m_climberProfile.calculate(Constants.schedulerPeriodTime, m_climberState, m_climberSetpoint);

    HardLimitDirection limit = m_climberLimiter.check();
    double climberPosition = m_climberLeftMotor.getEncoder().getPosition();
    if (limit == HardLimitDirection.kFree) {
      double climberFeedForward = m_climberFeedForwardController.calculate(m_climberState.velocity);
      m_climberLeftMotor.getClosedLoopController().setSetpoint(m_climberState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0,
          climberFeedForward);
    } else if (limit == HardLimitDirection.kForward) {
      if (m_climberLeftMotor.getAppliedOutput() > 0) {
        m_climberLeftMotor.set(0);
      }
      if (!MathUtil.isNear(Constants.ClimberConstants.kClimberUpperLimitInches, climberPosition,
            Constants.ClimberConstants.kClimberToleranceInches)) {
        m_climberLeftMotor.getEncoder().setPosition(Constants.ClimberConstants.kClimberUpperLimitInches);
      }
      if (m_climberSetpoint.position > climberPosition) {
        m_climberState = new TrapezoidProfile.State(Constants.ClimberConstants.kClimberUpperLimitInches, 0.0);
        m_climberSetpoint = new TrapezoidProfile.State(Constants.ClimberConstants.kClimberUpperLimitInches, 0.0);
      }
    } else if (limit == HardLimitDirection.kReverse) {
      if (m_climberLeftMotor.getAppliedOutput() < 0) {
        m_climberLeftMotor.set(0);
      }
      if (!MathUtil.isNear(Constants.ClimberConstants.kClimberLowerLimitInches, climberPosition,
            Constants.ClimberConstants.kClimberToleranceInches)) {
        m_climberLeftMotor.getEncoder().setPosition(Constants.ClimberConstants.kClimberLowerLimitInches);
      }
      if (m_climberSetpoint.position<climberPosition) {
        m_climberState = new TrapezoidProfile.State(Constants.ClimberConstants.kClimberLowerLimitInches, 0.0);
        m_climberSetpoint = new TrapezoidProfile.State(Constants.ClimberConstants.kClimberLowerLimitInches, 0.0);
      }
    }

    double climberFeedForward = m_climberFeedForwardController.calculate(m_climberState.velocity);
    m_climberMotor.getClosedLoopController().setSetpoint(
        m_climberState.position, ControlType.kPosition,
        ClosedLoopSlot.kSlot0, climberFeedForward, ArbFFUnits.kVoltage);

    m_TDclimberCurrentOutput.set(m_climberMotor.getOutputCurrent());
    m_TDclimberMeasuredPosition.set(m_climberMotor.getEncoder().getPosition());
    m_TDclimberProfilePosition.set(m_climberState.position);

    super.periodic();
  }
}
