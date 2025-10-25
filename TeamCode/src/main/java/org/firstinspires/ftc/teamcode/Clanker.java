package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Clanker extends LinearOpMode {
    public DcMotor frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;
    @Override
    public void runOpMode(){
        frontRightMotor = hardwareMap.get(DcMotor.class, "frm");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "flm");
        backRightMotor = hardwareMap.get(DcMotor.class, "brm");
        backLeftMotor = hardwareMap.get(DcMotor.class, "blm");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()){
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double denominator = 2.0 * Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double flPower = (y + x + rx) / denominator;
            double frPower = (y - x - rx) / denominator;
            double blPower = (y - x + rx) / denominator;
            double brPower = (y + x - rx) / denominator;
            telemetry.addData("left stick y: ", gamepad1.left_stick_y);
            telemetry.addData("left stick x: ", gamepad1.left_stick_x);
            telemetry.addData("frPower: ", frPower);
            telemetry.addData("flPower: ", flPower);
            telemetry.addData("brPower: ", brPower);
            telemetry.addData("blPower: ", blPower);
            telemetry.addData("right stick x: ", gamepad1.right_stick_x);
            telemetry.update();
            frontRightMotor.setPower(frPower);
            frontLeftMotor.setPower(flPower);
            backRightMotor.setPower(brPower);
            backLeftMotor.setPower(blPower);
        }
    }
}
