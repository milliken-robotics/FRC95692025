// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 2;
    public static final double DEADBAND = 0.1; 
    public static final double TURN_FACTOR = 0.5; 
  }
  public static class SwerveConstants {
    public static final double MAX_SPEED = Units.feetToMeters(10.5);;
    public static final double SCALE_TRANSLATION = 1; 
  }
  public static class HardwareMap {
    public static final int IT_ELEVATOR_L = 40;
    public static final int IT_ELEVATOR_R = 41;
    public static final int IT_CORAL_L = 53;
    public static final int IT_CORAL_R = 52;
    public static final int IT_BEAMBREAK2 = 9;
    public static final int IT_BEAMBREAK1 = 8;
    public static final int IT_ALGAE = 38;
    public static final int IT_ALGAE_PIVOT = 39;
  }

  public static class PIDVALUES {
  }

  public static class ElevatorConstnats {
    public static final double L3 = 880; 
    public static final double L2 = 250; 
    public static final double L1 = 30; 
    public static final double zerod = 25; 

  }

  public class AutoAlignCoordinates {

    public static class L {
        public static final Pose2d pose6 = new Pose2d(13.65, 2.8, new Rotation2d(Math.toRadians(-60)));
        public static final Pose2d pose7 = new Pose2d(14.38, 3.91, new Rotation2d(0));
        public static final Pose2d pose8 = new Pose2d(13.84, 5.14, new Rotation2d(Math.toRadians(60)));
        public static final Pose2d pose9 = new Pose2d(12.54, 5.2, new Rotation2d(Math.toRadians(120)));
        public static final Pose2d pose10 = new Pose2d(11.74, 4.14, new Rotation2d(Math.toRadians(180)));
        public static final Pose2d pose11 = new Pose2d(12.30, 2.98, new Rotation2d(Math.toRadians(-120)));
        public static final Pose2d pose17 = new Pose2d(3.92, 2.83, new Rotation2d(Math.toRadians(-120)));
        public static final Pose2d pose18 = new Pose2d(3.09, 4.01, new Rotation2d(Math.toRadians(180)));
        public static final Pose2d pose19 = new Pose2d(3.76, 4.98, new Rotation2d(Math.toRadians(120)));
        public static final Pose2d pose20 = new Pose2d(5., 5.17, new Rotation2d(Math.toRadians(60)));
        public static final Pose2d pose21 = new Pose2d(5.84, 4.14, new Rotation2d(0));
        public static final Pose2d pose22 = new Pose2d(5.293, 2.983, new Rotation2d(Math.toRadians(-60)));
    }

    public static class R {
        public static final Pose2d pose6 = new Pose2d(13.90, 2.96, new Rotation2d(Math.toRadians(-60)));
        public static final Pose2d pose7 = new Pose2d(14.38, 4.18, new Rotation2d(0));
        public static final Pose2d pose8 = new Pose2d(13.57, 5.24, new Rotation2d(Math.toRadians(60)));
        public static final Pose2d pose9 = new Pose2d(12.26, 5.07, new Rotation2d(Math.toRadians(120)));
        public static final Pose2d pose10 = new Pose2d(11.74, 3.85, new Rotation2d(Math.toRadians(180)));
        public static final Pose2d pose11 = new Pose2d(12.56, 2.8, new Rotation2d(Math.toRadians(-120)));
        public static final Pose2d pose17 = new Pose2d(4.27, 2.69, new Rotation2d(Math.toRadians(-120)));
        public static final Pose2d pose18 = new Pose2d(3.117, 3.545, new Rotation2d(Math.toRadians(180)));
        public static final Pose2d pose19 = new Pose2d(3.42, 4.89, new Rotation2d(Math.toRadians(120)));
        public static final Pose2d pose20 = new Pose2d(4.72, 5.37, new Rotation2d(Math.toRadians(60)));
        public static final Pose2d pose21 = new Pose2d(5.83, 4.52, new Rotation2d(0));
        public static final Pose2d pose22 = new Pose2d(5.19, 2.9, new Rotation2d(Math.toRadians(-60)));
    }

}
}