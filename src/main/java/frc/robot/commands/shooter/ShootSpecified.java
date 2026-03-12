package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import frc.robot.testingdashboard.Command;

public class ShootSpecified extends Command {
    private final Shooter m_Shooter;

    private final double m_flywheel;
    private final double m_hood;
    private final double m_turret;

    public ShootSpecified(double flywheel, double hood, double turret) {
        super(Shooter.getInstance(), "Shooting", "ShootSpecified");

        m_Shooter = Shooter.getInstance();

        m_flywheel = flywheel;
        m_hood = hood;
        m_turret = turret;

        addRequirements(m_Shooter);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        m_Shooter.setFlywheelTarget(m_flywheel);
        m_Shooter.setHoodTarget(m_hood);
        m_Shooter.setTurretTarget(m_turret, 0);
    }

    @Override
    public void end(boolean interrupted) {
        m_Shooter.setHoodTarget(0);
        m_Shooter.setFlywheelTarget(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
