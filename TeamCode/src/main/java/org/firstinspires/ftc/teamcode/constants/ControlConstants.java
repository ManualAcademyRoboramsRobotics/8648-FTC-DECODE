package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ControlConstants {
    //http://192.168.43.1:8080/dash

    //////////////////////////////////////////////////////////////
    // Constants
    //////////////////////////////////////////////////////////////
    public static double FEEDER_STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    public static double FEEDER_FULL_SPEED = 1.0;
    public static double FEEDER_BACKFEED_SPEED = -1;
    public static double LAUNCHER_BACKFEED_SPEED = -0.75;
    public static long FEED_TIME_MS = 800; //The feeder servos run this long when a shot is requested.
    public static long BACKFEED_TIME_MS = 650; //The feeder servos run this long when a shot is requested.

    public static double ALLOWED_VELOCITY_DIVERSION = 100;

    public static double LAUNCHER_KP = 300;
    public static double LAUNCHER_KI = 0;
    public static double LAUNCHER_KD = 0;
    public static double LAUNCHER_KF = 10;

    public static double DIVERTER_RIGHT = 0.4;
    public static double DIVERTER_LEFT = 0.64;

    public static double CLOSE_LAUNCH_SPEED = 1300;
    public static double FAR_LAUNCH_SPEED = 1700;

    public static double BLUE_GOAL_X = -70;
    public static double BLUE_GOAL_Y = 70;
    public static double RED_GOAL_X = 70;
    public static double RED_GOAL_Y = 70;

    public static double STICK_THRESHOLD = 0.05;

    public enum Alliance
    {
        RED,
        BLUE
    }
    public static Alliance CURRENT_ALLIANCE = Alliance.RED;

}
