package frc.robot.utils.structlogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.struct.Pose2dStruct;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

public class Pose2dPublisher implements NTStructPublisher<Pose2d> {
    private final NetworkTableInstance m_ntable;
    private final StructPublisher<Pose2d> m_publisher;

    private String m_name;

    private boolean m_print;

    public Pose2dPublisher(String name, Pose2dStruct defaultVal) {
        m_ntable = NetworkTableInstance.getDefault();
        m_name = name;
        m_publisher = m_ntable.getStructTopic(m_name, defaultVal).publish();
        m_print = false;
    }

    private Pose2d m_currentVal;
    private Pose2d m_lastVal;

    public void set(Pose2d value) {
        m_currentVal = value;

        if (m_print) {
            System.out.println(m_name + ": " + value);
        }
    }

    public Pose2d get() {
        return m_currentVal;
    }

    public void post() {
        if (m_lastVal != m_currentVal) {
            m_publisher.set(m_currentVal);
            m_lastVal = m_currentVal;
        }
    }

    public void setPrint(boolean print) {
        m_print = print;
    }
}
