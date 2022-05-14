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
public final class Constants 
{
    //Motor ID
    public static final int BACK_RIGHT_MOTOR = 2;

    //default PID values
    public static final double DEFAULT_P = 0.000001;
    public static final double DEFAULT_I = 0.0;
    public static final double DEFAULT_D = 0.000;

    //Default FF
    public static final double DEFAULT_S = 0.0;
    public static final double DEFAULT_V = 0.000202;

    //Default max velocity
    public static final double DEFAULT_VEL = 500;

    //Circumference
    public static final double CIRCUMFERENCE = Units.inchesToMeters(6 * Math.PI);

    // Joystick port
    public static final int JOYSTICK = 0;

    //electrical constants
    public static final double ramp_rate = 0.2;
    public static final double voltage_comp = 12.0;
    public static final int current_limit = 60;



}
