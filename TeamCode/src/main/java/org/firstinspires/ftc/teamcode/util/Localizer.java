package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Localizer {
    private PIDController xPIDControl;
    private PIDController yPIDControl;
    private PIDController hPIDControl;

    public Localizer(PIDCoefficients xCoefficients, PIDCoefficients yCoefficients, PIDCoefficients hCoefficients) {
        xPIDControl = new PIDController(xCoefficients);
        yPIDControl = new PIDController(yCoefficients);
        hPIDControl = new PIDController(hCoefficients);
    }

    private void moveRobotTo(Pose2D desiredPose) {

    }
}
