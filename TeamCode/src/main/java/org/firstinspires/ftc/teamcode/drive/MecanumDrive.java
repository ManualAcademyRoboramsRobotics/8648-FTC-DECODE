package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MecanumDrive extends BaseDrive {

    protected DcMotorEx m_LeftFrontDrive, m_LeftBackDrive, m_RightFrontDrive, m_RightBackDrive;

    public MecanumDrive(DcMotorEx leftFrontMotor, DcMotorEx leftBackMotor, DcMotorEx rightFrontMotor, DcMotorEx rightBackMotor) {
        m_LeftFrontDrive = leftFrontMotor;
        m_LeftBackDrive = leftBackMotor;
        m_RightFrontDrive = rightFrontMotor;
        m_RightBackDrive = rightBackMotor;
    }

    @Override
    public void Drive(double ForwardPower, double StrafePower, double TurnPower, double PowerFactor) {
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]

        double denominator = Math.max(Math.abs(ForwardPower) + Math.abs(StrafePower) + Math.abs(TurnPower), 1);

        double leftFrontPower = ((ForwardPower + StrafePower + TurnPower) / denominator) * PowerFactor;
        double leftBackPower = ((ForwardPower - StrafePower + TurnPower) / denominator) * PowerFactor;
        double rightFrontPower = ((ForwardPower - StrafePower - TurnPower) / denominator) * PowerFactor;
        double rightBackPower = ((ForwardPower + StrafePower - TurnPower) / denominator) * PowerFactor;

        m_LeftFrontDrive.setPower(leftFrontPower);
        m_LeftBackDrive.setPower(leftBackPower);
        m_RightFrontDrive.setPower(rightFrontPower);
        m_RightBackDrive.setPower(rightBackPower);
    }

    @Override
    public void Drive (double ForwardPower, double StrafePower, double TurnPower) {
        Drive(ForwardPower, StrafePower, TurnPower, 1);
    }
}
