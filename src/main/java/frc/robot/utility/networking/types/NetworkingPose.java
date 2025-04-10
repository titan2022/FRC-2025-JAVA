package frc.robot.utility.networking.types;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class NetworkingPose {
    public String objectName;
    public Translation3d position;
    public Rotation3d rotation;
    public double distance;

    public NetworkingPose(String objectName, Translation3d position, Rotation3d rotation, double distance) {
        this.objectName = objectName;
        this.position = position;
        this.rotation = rotation;
        this.distance = distance;
    }

    public String toString() {
        return position.toString() + " " + rotation.toString();
    }
}