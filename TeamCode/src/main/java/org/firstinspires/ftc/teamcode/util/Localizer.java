package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Localizer {
    private PIDController xPIDControl;
    private PIDController yPIDControl;
    private PIDController hPIDControl;

    public Localizer(PIDCoefficients Coefficients) {
        xPIDControl = new PIDController(Coefficients);
        yPIDControl = new PIDController(Coefficients);
        hPIDControl = new PIDController(Coefficients);
    }

    private void moveRobotTo(Pose2D desiredPose) {

    }
}
