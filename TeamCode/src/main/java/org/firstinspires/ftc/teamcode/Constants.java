package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    //http://192.168.43.1:8080/dash
    public static double Kp_Lateral = 0.02; //0.0002;
    public static double Ki_Lateral = 0.0;
    public static double Kd_Lateral = 0.0;
    public static double Kp_Heading = 0.008; //0.01;
    public static double Ki_Heading = 0.0;
    public static double Kd_Heading = 0.0;

    public static double X_Desired = 0.0;
    public static double Y_Desired = 0.0;
    public static double H_Desired = 0.0;

    public static double X_Offset_in = 1.0;
    public static double Y_Offset_in = -4.75;


}
