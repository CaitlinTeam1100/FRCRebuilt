package frc.robot.commands.shooter;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.LED.LEDPattern;
import frc.robot.subsystems.LED.LEDSection.Priority;
import frc.robot.testingdashboard.Command;
import frc.robot.testingdashboard.TDSendable;
import frc.robot.utils.Configuration;
import frc.robot.utils.TrajectorySolver;
import frc.robot.utils.TrajectorySolver.SolveType;
import frc.robot.utils.TrajectorySolver.TrajectoryConditions;
import frc.robot.utils.TrajectorySolver.TrajectoryParameters;
import frc.robot.utils.structlogging.StructLogger;
import frc.robot.utils.vision.VisionEstimationResult;

public class ShootToPose extends Command {
    private final Shooter m_Shooter;
    private final Vision m_Vision;
    private final Drive m_Drive;

    private Transform3d m_cameraToTurret;
    private Transform3d m_chassisToTurret;

    private final Supplier<Pose3d> m_targetSupplier;

    private Field2d m_trajectoryDisplay;
    private final int m_displayRes;

    private String m_cameraName;

    private StructLogger[] m_ballPoseLogger;
    private Translation3d[] m_ballPosition;
    private Translation3d[] m_ballVelocity;
    private double m_lastShotTime;

    private final InterpolatingDoubleTreeMap m_treeMap;

    public ShootToPose(Supplier<Pose3d> targetSupplier) {
        super(Shooter.getInstance(), "Targeted Shooting", "ShootToPose");

        m_treeMap = new InterpolatingDoubleTreeMap();
        m_treeMap.put( 0.0, Math.toRadians(65));
        m_treeMap.put( 2.7, Math.toRadians(65));
        m_treeMap.put( 2.71, Math.toRadians(50));
        m_treeMap.put(16.0, Math.toRadians(50));


        m_Shooter = Shooter.getInstance();
        m_Vision = Vision.getInstance();
        m_Drive = Drive.getInstance();
        m_targetSupplier = targetSupplier;

        m_displayRes = 32;

        m_cameraName = "TurretCamera";

        if (RobotBase.isSimulation()) {
            m_ballPoseLogger = new StructLogger[16];
            m_ballPosition = new Translation3d[16];
            m_ballVelocity = new Translation3d[16];
            for (int i = 0; i < 16; i++) {
                m_ballPoseLogger[i] = StructLogger.pose3dLogger(m_Shooter, "BallPoses/Pose_" + i, null);
            }
        }

        // Drive/Vision isn't a requirement - it's used for reading only
        addRequirements(m_Shooter);
    }

    @Override 
    public void initialize() {
        Configuration cfg = Configuration.getInstance();
        m_chassisToTurret = new Transform3d(
            new Pose3d(),
            new Pose3d(
                new Translation3d(
                    cfg.getDouble("Shooter", "turretPositionX"),
                    cfg.getDouble("Shooter", "turretPositionY"),
                    cfg.getDouble("Shooter", "turretPositionZ")
                ), Rotation3d.kZero)
        );
    }

    @Override
    public void execute() {
        Pose3d target = m_targetSupplier.get();
        if (target == null) return;

        Configuration cfg = Configuration.getInstance();
        Optional<VisionEstimationResult> result = m_Vision.getLatestFromCamera(m_cameraName);
        Pose3d turretPose;
        //if (result.isPresent()) {
        turretPose = m_Shooter.getTurretPose();
        //} else {
            //Pose3d chassisPose = new Pose3d(m_Drive.getPose());
            //turretPose = chassisPose.transformBy(m_chassisToTurret);
        //}

        double dist = turretPose.getTranslation().toTranslation2d().minus(target.getTranslation().toTranslation2d()).getDistance(Translation2d.kZero);
        double angle = m_treeMap.get(dist);

        ChassisSpeeds chassisVelocity = m_Drive.getMeasuredSpeeds();

        TrajectoryConditions conditions = new TrajectoryConditions();
        conditions.start = turretPose;
        conditions.target = target;
        conditions.theta = angle;
        
        TrajectoryParameters params = TrajectorySolver.solveTrajectory(conditions, SolveType.CONTROL_THETA);

        Translation3d baseVelocity = new Translation3d(params.velocity,0,0).rotateBy(new Rotation3d(0, -params.theta_pitch, Math.PI + params.theta_yaw));
        Translation2d chassis = new Translation2d(chassisVelocity.vxMetersPerSecond, chassisVelocity.vyMetersPerSecond);
        chassis = chassis.rotateBy(m_Drive.getPose().getRotation().unaryMinus());
        Translation3d correctedVelocity = baseVelocity;//.minus(new Translation3d(chassis));

        double yaw = correctedVelocity.toTranslation2d().getAngle().getRadians();
        double pitch = new Translation2d(correctedVelocity.toTranslation2d().getDistance(Translation2d.kZero), correctedVelocity.getZ()).getAngle().getRadians();
        double velocity = correctedVelocity.getDistance(Translation3d.kZero);

        double turretYaw = yaw;
        double hoodTarget = m_Shooter.pitchToHood(pitch);
        double flywheelRPM = m_Shooter.velocityToRPM(velocity, pitch);

        System.out.printf("Velocity: %f, RPM: %f\n", velocity, flywheelRPM);

        m_Shooter.setTurretTarget(turretYaw, 0);
        m_Shooter.setHoodTarget(hoodTarget);
        m_Shooter.setFlywheelTarget(flywheelRPM);

        if (RobotBase.isSimulation()) {
            int last = -1;
            for (int i = 0; i < m_ballPoseLogger.length; i++) {
                if (m_ballPosition[i] == null || m_ballPosition[i].getZ() < 0) {
                    last = i;
                    break;
                }
            }

            if (Timer.getFPGATimestamp() > m_lastShotTime + 0.2 && last > -1) {
                m_lastShotTime = Timer.getFPGATimestamp();
                m_ballPosition[last] = turretPose.getTranslation();
                double simHoodAngle = m_Shooter.hoodToPitch(m_Shooter.getHoodTarget());
                double simFlywheelVelocity = m_Shooter.RPMtoVelocity(m_Shooter.getFlywheelTarget(), simHoodAngle);
                double simTurretTarget = m_Shooter.getTurretTarget();
                m_ballVelocity[last] = new Translation3d(simFlywheelVelocity,0,0).rotateBy(new Rotation3d(
                    0,
                    -simHoodAngle,
                    simTurretTarget
                ));

                Translation2d chassisVelocity2d = new Translation2d(chassisVelocity.vxMetersPerSecond, chassisVelocity.vyMetersPerSecond);
                chassisVelocity2d = chassisVelocity2d.rotateBy(m_Drive.getPose().getRotation().unaryMinus());
                m_ballVelocity[last] = m_ballVelocity[last].plus(new Translation3d(chassisVelocity2d));
            }

            for (int i = 0; i < m_ballPoseLogger.length; i++) {
                if (m_ballPosition[i] == null) continue;

                m_ballVelocity[i] = m_ballVelocity[i].plus(new Translation3d(0, 0, -9.8*Constants.schedulerPeriodTime));
                m_ballPosition[i] = m_ballPosition[i].plus(m_ballVelocity[i].times(Constants.schedulerPeriodTime));

                Pose3d pose = new Pose3d(m_ballPosition[i], Rotation3d.kZero);
                m_ballPoseLogger[i].setStruct(pose);
            }

        }

        //System.out.println(m_Shooter.turretAtTarget() + " " + m_Shooter.flywheelAtTarget() + " " + m_Shooter.hoodAtTarget());
        //if (m_Shooter.turretAtTarget() && m_Shooter.flywheelAtTarget() && m_Shooter.hoodAtTarget()) {
			//LED.getInstance().setPattern(1, LEDPattern.kCheckeredBlinkGreen, Priority.INFO);
            m_Shooter.chimneySpeed(1);
        //} else {
			//LED.getInstance().setPattern(1, LEDPattern.kCheckeredBlinkYellow, Priority.INFO);
            //m_Shooter.chimneyStop();
        //}

        m_Shooter.updateTrajectoryDisplay(conditions, params);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_Shooter.clearTrajectoryDisplay();

        m_Shooter.setFlywheelTarget(0);
        m_Shooter.setHoodTarget(0);
        m_Shooter.chimneyStop();
    }
}
