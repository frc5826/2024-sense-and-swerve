// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.io.IOException;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

    public static final double driveDeadBand = 0.15;
    public static final double turnDeadBand = 0.15;

    public static final Pose2d speakerPark = new Pose2d(1.65, 5.54, Rotation2d.fromDegrees(180));
    public static final Pose2d ampPark = new Pose2d(1.84, 7.5, Rotation2d.fromDegrees(90));

    public static final Pose2d leftStagePark = new Pose2d(4.5, 5, Rotation2d.fromDegrees(300));
    public static final Pose2d centerStagePark = new Pose2d(5.8, 4.5, Rotation2d.fromDegrees(180));
    public static final Pose2d rightStagePark = new Pose2d(4.5, 3.5, Rotation2d.fromDegrees(60));

}
