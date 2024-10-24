package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveConstants {

    public static double TICKS_PER_REV = 383.6;
    public static double WHEEL_DIAMETER = 0.1;
    public static double DISTANCE_PER_PULSE = WHEEL_DIAMETER * Math.PI / TICKS_PER_REV;
    //TODO: Check units
    public static double TRACK_WIDTH = 6.75;

    public static double CENTER_WHEEL_OFFSET = 6;

    public static double MAX_VELOCITY = 1.5;
    public static double MAX_ACCELERATION = 1.5;

    public static double B = 2.0;
    public static double ZETA = 0.7;


    public static int armSamplePick = 1350;
    public static int armSampleRest = 1150;
    public static int armSampleScore = 790;

    public static int armZero = 200;

    public static int armSpecimenPick = armSamplePick;
    public static int armSpecimenOver = 875;
    public static int armSpecimenClip = 875;
    public static int armSpecimenRest = 760;

    public static int armStartHang = 875;
    public static int armFinishHang = 125;

    public static int armCurrent;

    public static boolean leftBumperPressed = false;
    public static boolean rightBumperPressed = false;

    public static boolean huskyReading = false;


}
