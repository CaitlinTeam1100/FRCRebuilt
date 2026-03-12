package frc.robot.utils.structlogging;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.struct.Pose3dStruct;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

public class Pose3dPublisher implements NTStructPublisher<Pose3d> {
    private final NetworkTableInstance m_ntable;
    private final StructPublisher<Pose3d> m_publisher;

    private String m_name;

    private boolean m_print;

    public Pose3dPublisher(String name, Pose3dStruct defaultVal) {
        m_ntable = NetworkTableInstance.getDefault();
        m_name = name;
        m_publisher = m_ntable.getStructTopic(m_name, defaultVal).publish();
        m_print = false;
    }

    private Pose3d m_currentVal;
    private Pose3d m_lastVal;

    public void set(Pose3d value) {
        m_currentVal = value;

        if (m_print) {
            System.out.println(m_name + ": " + value);
        }
    }

    public Pose3d get() {
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
