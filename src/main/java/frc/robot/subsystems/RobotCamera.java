package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.photonvision.PhotonCamera;

public class RobotCamera {

    private final Transform3d robotLocation;
    private final PhotonCamera camera;
    private final boolean aprilTag;

    public RobotCamera(Translation3d cameraLocation, Rotation3d cameraDirection, String cameraName, boolean aprilTag) {
        this.robotLocation = new Transform3d(cameraLocation, cameraDirection);
        this.camera = new PhotonCamera(cameraName);
        this.aprilTag = aprilTag;
    }

    public Transform3d getRobotLocation() {
        return robotLocation;
    }

    public PhotonCamera getCamera() {
        return camera;
    }

    public boolean isAprilTag() {
        return aprilTag;
    }

    @Override
    public String toString() {
        return "RobotCamera{" +
                "robotLocation=" + robotLocation +
                ", camera=" + camera.getName() +
                ", aprilTag=" + aprilTag +
                '}';
    }
}
