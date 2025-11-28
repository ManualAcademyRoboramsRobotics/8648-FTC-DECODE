package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;

//@Disabled
@TeleOp(name = "Drive Motor Direction Debugger")
public class EncoderOffsetDebugger extends BaseOpMode {

    @Override
    public void init() {
        super.init();
        m_LeftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m_LeftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m_RightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m_RightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        m_Pinpoint.update();

        m_LeftFrontDrive.setPower(0);
        m_LeftBackDrive.setPower(0);
        m_RightFrontDrive.setPower(0);
        m_RightBackDrive.setPower(0);

        telemetry.addData("INSTRUCTIONS:", "SPIN ROBOT AROUND TRACKING POINT - HEADING SHOULD REFLECT ACCORDINGLY. ENSURE x_enc AND y_enc DO NOT GO ABOVE ~4IN");
        telemetry.addData("x_enc", m_Pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("y_enc", m_Pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("h_enc", m_Pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }
}
