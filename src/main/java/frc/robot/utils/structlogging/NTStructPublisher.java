package frc.robot.utils.structlogging;

import edu.wpi.first.util.struct.StructSerializable;

public interface NTStructPublisher<S extends StructSerializable> {
    public void post();

    public S get();
    public void set(S value);

    public void setPrint(boolean print);
}