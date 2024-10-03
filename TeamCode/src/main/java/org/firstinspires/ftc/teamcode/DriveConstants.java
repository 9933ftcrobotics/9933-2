package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveConstants {

    public static double TICKS_PER_REV = 383.6;
    public static double WHEEL_DIAMETER = 0.1;
    public static double DISTANCE_PER_PULSE = WHEEL_DIAMETER * Math.PI / TICKS_PER_REV;
    public static double TRACK_WIDTH = 0.4572;

    public static double MAX_VELOCITY = 1.5;
    public static double MAX_ACCELERATION = 1.5;

    public static double B = 2.0;
    public static double ZETA = 0.7;

}
