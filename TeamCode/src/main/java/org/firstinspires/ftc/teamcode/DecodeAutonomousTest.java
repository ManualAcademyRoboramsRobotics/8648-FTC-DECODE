package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.PIDController;
import com.acmerobotics.dashboard.FtcDashboard;


@Autonomous (name = "PID Tuning")
public class DecodeAutonomousTest extends OpMode {
    //http://192.168.43.1:8080/dash
    private FtcDashboard dashboard = null;
    protected DcMotorEx leftFrontDrive = null;
    protected DcMotorEx leftBackDrive = null;
    protected DcMotorEx rightFrontDrive = null;
    protected DcMotorEx rightBackDrive = null;
    protected MecanumDrive mecanumDrive = null;
    private IMU imu = null;
    private PIDController xPIDControl = null;
    private PIDController yPIDControl = null;
    private PIDController hPIDControl = null;

//    final PIDCoefficients YCoefficients = new PIDCoefficients(0.0002,0,0);
    final PIDCoefficients YCoefficients = new PIDCoefficients(Constants.Kp_Lateral, Constants.Ki_Lateral, Constants.Kd_Lateral);
//    final PIDCoefficients HCoefficients = new PIDCoefficients(0.01,0,0);
    final PIDCoefficients HCoefficients = new PIDCoefficients(Constants.Kp_Heading, Constants.Ki_Heading, Constants.Kd_Heading);

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "lfd");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "lbd");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rfd");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rbd");
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Initial Directions
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        mecanumDrive = new MecanumDrive(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive);

        xPIDControl = new PIDController(YCoefficients);
        yPIDControl = new PIDController(YCoefficients);
        hPIDControl = new PIDController(HCoefficients);


        telemetry.addData("Status", "Odometer Initialized");
        }

    @Override
    public void init_loop(){
        telemetry.addData("y_enc", -leftFrontDrive.getCurrentPosition());
        telemetry.addData("x_enc", -leftBackDrive.getCurrentPosition());
    }

    @Override
    public void loop() {
        Pose2D DesiredPose = new Pose2D(DistanceUnit.INCH, 15,10, AngleUnit.DEGREES, 45);
        double y = yPIDControl.pidControl(-leftFrontDrive.getCurrentPosition(), DesiredPose.getX(DistanceUnit.INCH));
        double x = xPIDControl.pidControl(-leftBackDrive.getCurrentPosition(), DesiredPose.getX(DistanceUnit.INCH));

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double h = hPIDControl.pidControl(angles.getYaw(AngleUnit.DEGREES), DesiredPose.getHeading(AngleUnit.DEGREES));

        mecanumDrive.drive(y, x, h);

        telemetry.addData("X_PID", 0);
        telemetry.addData("Y_PID", yPIDControl);
        telemetry.addData("H_PID", h);
        telemetry.addData("Heading", angles.getYaw(AngleUnit.DEGREES));
        telemetry.addData("y_enc", -leftFrontDrive.getCurrentPosition());
        telemetry.addData("x_enc", -leftBackDrive.getCurrentPosition());

//        telemetry.addData("X", pinpointDriver.getPosX(DistanceUnit.INCH));
//        telemetry.addData("Y", pinpointDriver.getPosY(DistanceUnit.INCH));
//        telemetry.addData("Heading", pinpointDriver.getHeading(AngleUnit.DEGREES));
//        telemetry.addData("Position", pinpointDriver.getPosition());
//        telemetry.addData("Status", pinpointDriver.getDeviceStatus());
//        telemetry.addData("Device Version Number:", pinpointDriver.getDeviceVersion());
//        telemetry.addData("Heading Scalar", pinpointDriver.getYawScalar());
        telemetry.update();


    }
}
