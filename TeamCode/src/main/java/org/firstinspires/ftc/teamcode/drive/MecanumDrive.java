package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MecanumDrive extends BaseDrive {

    protected DcMotorEx m_LeftFrontDrive = null;
    protected DcMotorEx m_LeftBackDrive = null;
    protected DcMotorEx m_RightFrontDrive = null;
    protected DcMotorEx m_RightBackDrive = null;

    MecanumDrive(DcMotorEx leftFrontMotor, DcMotorEx leftBackMotor, DcMotorEx rightFrontMotor, DcMotorEx rightBackMotor) {
        m_LeftFrontDrive = leftFrontMotor;
        m_LeftBackDrive = leftBackMotor;
        m_RightFrontDrive = rightFrontMotor;
        m_RightBackDrive = rightBackMotor;
    }
    @Override
    public void Drive(double Forward, double Strafe, double Turn) {
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]

        double denominator = Math.max(Math.abs(Forward) + Math.abs(Strafe) + Math.abs(Turn), 1);

        double leftFrontPower = (Forward + Strafe + Turn) / denominator;
        double leftBackPower = (Forward - Strafe + Turn) / denominator;
        double rightFrontPower = (Forward - Strafe - Turn) / denominator;
        double rightBackPower = (Forward + Strafe - Turn) / denominator;

        m_LeftFrontDrive.setPower(leftFrontPower);
        m_LeftBackDrive.setPower(leftBackPower);
        m_RightFrontDrive.setPower(rightFrontPower);
        m_RightBackDrive.setPower(rightBackPower);
    }
}
