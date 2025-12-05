package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveConstants {
    //http://192.168.43.1:8080/dash
    public static double XY_KP = 0.068; //0.02 minibot
    public static double XY_KI = 0.0;
    public static double XY_KD = 0.0;

    public static double H_KP = 0.01; //0.008 minibot
    public static double H_KI = 0.0;
    public static double H_KD = 0.0;

    public static double X_Desired = 0.0;
    public static double Y_Desired = 0.0;
    public static double H_Desired = 0.0;

    public static double XY_LOCALIZER_TOLERANCE_IN = 4;
    public static double H_LOCALIZER_TOLERANCE_DEGREE = 5;

    public static double ODOMETERY_Y_OFFSET = -1.5; // 1.0 minibot
    public static double ODOMETERY_X_OFFSET = -0.875; // -4.75 minibot

    public static double SLOW_SPEED = 0.75;
    public static double FULL_SPEED = 1;

    public static double SINX = 1;
    public static double COSY = 1;
    public static double SINY = -1;
    public static double COSX = 1;
}
