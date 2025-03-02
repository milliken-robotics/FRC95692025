// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int IT_CORAL_R = 2;
    public static final int IT_BEAMBREAK = 9;
    public static final int IT_ALGAE = 38;
    public static final int IT_ALGAE_PIVOT = 39;
  }

  public static class PIDVALUES {
  }

  public static class ElevatorConstnats {
    public static final double L3 = 900; 
    public static final double L2 = 250; 
    public static final double L1 = 30; 
    public static final double zerod = 25; 

  }
}
