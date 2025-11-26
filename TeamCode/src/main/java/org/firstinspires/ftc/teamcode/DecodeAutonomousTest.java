package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.Drawable;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.PIDController;
import com.acmerobotics.dashboard.FtcDashboard;
import com.sun.tools.javac.code.Attribute;


@Autonomous (name = "First Autonomous Test")
public class DecodeAutonomousTest extends BaseOpMode {
    private PIDController xPIDControl = null;
    private PIDController yPIDControl = null;
    private PIDController hPIDControl = null;

    final PIDCoefficients YCoefficients = new PIDCoefficients(Constants.XY_KP, Constants.XY_KI, Constants.XY_KD);
    final PIDCoefficients XCoefficients = new PIDCoefficients(Constants.XY_KP, Constants.XY_KI, Constants.XY_KD);
    final PIDCoefficients HCoefficients = new PIDCoefficients(Constants.H_KP, Constants.H_KI, Constants.H_KD);

    @Override
    public void init() {
        super.init();
        xPIDControl = new PIDController(XCoefficients);
        yPIDControl = new PIDController(YCoefficients);
        hPIDControl = new PIDController(HCoefficients);

        telemetry.addData("Status", "Odometer Initialized");
        }

    @Override
    public void init_loop(){
        pinpoint.update();
        telemetry.addData("x_enc", pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("y_enc", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("h_enc", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("x_offset", pinpoint.getXOffset(DistanceUnit.INCH));
        telemetry.addData("y_offset", pinpoint.getYOffset(DistanceUnit.INCH));
        telemetry.addData("Position", pinpoint.getPosition());
    }

    @Override
    public void loop() {
        pinpoint.update();

        Pose2D DesiredPose = new Pose2D(DistanceUnit.INCH, Constants.X_Desired, Constants.Y_Desired, AngleUnit.DEGREES, Constants.H_Desired);
        // Pose2D CurrentPose = new Pose2D(DistanceUnit.INCH, -pinpoint.getPosY(DistanceUnit.INCH), pinpoint.getPosX(DistanceUnit.INCH), AngleUnit.DEGREES, -pinpoint.getHeading(AngleUnit.DEGREES));
        // Pinpoint X and Y are opposite of our reference (Y Forward/Back, X Left/Right)
        double y = yPIDControl.pidControl(pinpoint.getPosX(DistanceUnit.INCH), DesiredPose.getY(DistanceUnit.INCH), Constants.XY_KP, Constants.XY_KI, Constants.XY_KD);
        double x = xPIDControl.pidControl(-pinpoint.getPosY(DistanceUnit.INCH), DesiredPose.getX(DistanceUnit.INCH), Constants.XY_KP, Constants.XY_KI, Constants.XY_KD);
        double h = hPIDControl.pidControl(-pinpoint.getHeading(AngleUnit.DEGREES), DesiredPose.getHeading(AngleUnit.DEGREES), Constants.H_KP, Constants.H_KI, Constants.H_KD);

        double cosine = Math.cos(pinpoint.getHeading(AngleUnit.RADIANS));
        double sine = Math.sin(pinpoint.getHeading(AngleUnit.RADIANS));

        double yOutput = (y * cosine) - (x * sine);
        double xOutput = (y * sine) + (x * cosine);

        mecanumDrive.drive(yOutput, xOutput, h);


        telemetry.addData("sine", sine);
        telemetry.addData("cosine", cosine);
        telemetry.addData("X_PID", x);
        telemetry.addData("Y_PID", y);
        telemetry.addData("H_PID", h);
        telemetry.addData("x_enc", pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("y_enc", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("h_enc", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("x_desired", DesiredPose.getX(DistanceUnit.INCH));
        telemetry.addData("y_desired", DesiredPose.getY(DistanceUnit.INCH));
        telemetry.addData("h_desired", DesiredPose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", pinpoint.getPosition());
        telemetry.update();


    }
}
