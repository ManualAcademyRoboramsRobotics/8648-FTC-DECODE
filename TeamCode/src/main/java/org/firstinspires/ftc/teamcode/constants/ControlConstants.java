package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ControlConstants {
    //http://192.168.43.1:8080/dash
    public static double LAUNCHER_KP = 300;
    public static double LAUNCHER_KI = 0;
    public static double LAUNCHER_KD = 0;
    public static double LAUNCHER_KF = 10;

    public static double DIVERTER_RIGHT = 0.295;
    public static double DIVERTER_LEFT = 0.355;

    public static double CLOSE_LAUNCH_SPEED = 1300;
    public static double FAR_LAUNCH_SPEED = 1700;
}
