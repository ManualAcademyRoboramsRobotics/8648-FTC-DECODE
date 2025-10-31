package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MecanumDrive {

    protected DcMotorEx leftFrontDrive = null;
    protected DcMotorEx leftBackDrive = null;
    protected DcMotorEx rightFrontDrive = null;
    protected DcMotorEx rightBackDrive = null;

    double denominator = 0;
    double leftFrontPower = 0;
    double leftBackPower = 0;
    double rightFrontPower = 0;
    double rightBackPower = 0;

    MecanumDrive(DcMotorEx leftFrontMotor, DcMotorEx leftBackMotor, DcMotorEx rightFrontMotor, DcMotorEx rightBackMotor) {
        leftFrontDrive = leftFrontMotor;
        leftBackDrive = leftBackMotor;
        rightFrontDrive = rightFrontMotor;
        rightBackDrive = rightBackMotor;
    }

    public void drive(double y, double x, double rx) {
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]

        denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        leftFrontPower = (y + x + rx) / denominator;
        leftBackPower = (y - x + rx) / denominator;
        rightFrontPower = (y - x - rx) / denominator;
        rightBackPower = (y + x - rx) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightFrontDrive.setPower(rightFrontPower);
        rightBackDrive.setPower(rightBackPower);
    }

}
