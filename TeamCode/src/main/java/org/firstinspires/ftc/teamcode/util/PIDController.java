package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private ElapsedTime timer;
    private double previousError;

    private double kP;
    private double kI;
    private double kD;

    private double iRunning;

    public PIDController(PIDCoefficients Coefficients) {
        kP = Coefficients.p;
        kI = Coefficients.i;
        kD = Coefficients.d;

        iRunning = 0;
        previousError = 0;

        timer = new ElapsedTime();
    }

    public PIDController(PIDFCoefficients Coefficients) {

    }

    public double pidControl(double Current, double Desired) {
        double elapsedTime = timer.time();
        timer.reset();

        double currentError = Desired - Current;

        double p = kP * currentError;
        double i = kI * (currentError * (elapsedTime));
        iRunning += i;

        //    if i > max_i:
        //    i = max_i
        //    elif i < -max_i:
        //    i = -max_i

        double d = kD * (currentError - previousError) / elapsedTime;

        previousError = currentError;

        return p + iRunning + d;
    }
}
