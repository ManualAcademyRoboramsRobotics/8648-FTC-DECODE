package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    final ElapsedTime m_Timer;
    private double m_PreviousError, m_RunningIntegral;
    private PIDCoefficients m_Coefficients;

    public PIDController(PIDCoefficients coefficients) {
        m_Coefficients = coefficients;

        m_RunningIntegral = 0;
        m_PreviousError = 0;

        m_Timer = new ElapsedTime();
    }

    public void SetCoefficients(PIDCoefficients coefficients) {
        m_Coefficients = coefficients;
    }

    public PIDCoefficients GetCoefficients() {
        return m_Coefficients;
    }

    public double pidControl(double Current, double Desired) {
        double elapsedTime = m_Timer.time();
        m_Timer.reset();

        double currentError = Desired - Current;

        double p = m_Coefficients.p * currentError;
        double i = m_Coefficients.i * (currentError * (elapsedTime));
        m_RunningIntegral += i;

        //    if i > max_i:
        //    i = max_i
        //    elif i < -max_i:
        //    i = -max_i

        double d = m_Coefficients.d * (currentError - m_PreviousError) / elapsedTime;

        m_PreviousError = currentError;

        return p + m_RunningIntegral + d;
    }
}
