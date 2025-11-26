package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.PIDController;


public abstract class BaseOpMode extends OpMode {
    //http://192.168.43.1:8080/dash
    final FtcDashboard dashboard = FtcDashboard.getInstance();
    protected DcMotorEx leftFrontDrive = null;
    protected DcMotorEx leftBackDrive = null;
    protected DcMotorEx rightFrontDrive = null;
    protected DcMotorEx rightBackDrive = null;
    protected MecanumDrive mecanumDrive = null;
    protected GoBildaPinpointDriver pinpoint = null;
    protected Localizer localizer = null;

    final PIDCoefficients XYCoefficients = new PIDCoefficients(Constants.XY_KP, Constants.XY_KI, Constants.XY_KD);
    final PIDCoefficients HCoefficients = new PIDCoefficients(Constants.H_KP, Constants.H_KI, Constants.H_KD);

    Pose2D DesiredPose = null;

    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "lfd");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "lbd");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rfd");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rbd");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pp");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Initial Directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        mecanumDrive = new MecanumDrive(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive);

        pinpoint.initialize();
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setOffsets(Constants.ODOMETERY_Y_OFFSET, Constants.ODOMETERY_X_OFFSET, DistanceUnit.INCH);
        pinpoint.resetPosAndIMU();


        localizer = new Localizer(mecanumDrive, Constants.XY_LOCALIZER_TOLERANCE, Constants.H_LOCALIZER_TOLERANCE, XYCoefficients, XYCoefficients, HCoefficients );
    }
}
