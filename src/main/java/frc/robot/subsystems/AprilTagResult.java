package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Robot;

public class AprilTagResult {

    private final Transform3d aprilTagLocation;
    private final int id;
    private final double ambiguity;

    private final RobotCamera camera;

    public AprilTagResult(RobotCamera camera, Transform3d aprilTagLocation, int id, double ambiguity) {
        this.camera = camera;
        this.aprilTagLocation = aprilTagLocation;
        this.id = id;
        this.ambiguity = ambiguity;
    }

    public Transform3d getAprilTagLocation() {
        return aprilTagLocation;
    }

    public int getId() {
        return id;
    }

    public double getAmbiguity() {
        return ambiguity;
    }

    @Override
    public String toString() {
        return "AprilTagResult{" +
                "aprilTagLocation=" + aprilTagLocation +
                ", id=" + id +
                ", ambiguity=" + ambiguity +
                ", camera=" + camera +
                '}';
    }
}
