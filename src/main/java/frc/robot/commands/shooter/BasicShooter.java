// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import frc.robot.testingdashboard.Command;
import frc.robot.utils.Configuration;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BasicShooter extends Command {
  private final Shooter m_Shooter;
  private final Spindexer m_spindexer;
  private double m_spindexerSpeed;

  public BasicShooter() {
    super(Shooter.getInstance(), "Basic","BasicShooter");
    m_Shooter = Shooter.getInstance();
    m_spindexer = Spindexer.getInstance();
    m_spindexerSpeed = Configuration.getInstance().getDouble("Spindex", "spindexSpeed");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Shooter.setFlywheelTarget(Constants.ShooterConstants.kBasicShooterRPM);
    m_Shooter.setHoodTarget(25);
    m_Shooter.setTurretTarget(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_Shooter.flywheelAtTarget() && m_Shooter.hoodAtTarget() && m_Shooter.turretAtTarget()){
      m_Shooter.chimneySpeed(Constants.ShooterConstants.kChimneySpeed);
      m_spindexer.spinIn(m_spindexerSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Shooter.setFlywheelTarget(0);
    m_Shooter.chimneyStop();
    m_spindexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
