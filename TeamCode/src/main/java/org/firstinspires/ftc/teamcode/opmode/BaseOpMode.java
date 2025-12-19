package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Localizer;


public abstract class BaseOpMode extends OpMode {
    //http://192.168.43.1:8080/dash
    final FtcDashboard m_WebDashboard = FtcDashboard.getInstance();
    protected DcMotorEx m_LeftFrontDrive, m_LeftBackDrive, m_RightFrontDrive, m_RightBackDrive;
    protected MecanumDrive m_MecanumDrive;
    protected GoBildaPinpointDriver m_Pinpoint;
    protected Localizer m_Localizer;
    public Pose2D m_StartingPosition = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    final PIDCoefficients XYCoefficients = new PIDCoefficients(DriveConstants.XY_KP, DriveConstants.XY_KI, DriveConstants.XY_KD);
    final PIDCoefficients HCoefficients = new PIDCoefficients(DriveConstants.H_KP, DriveConstants.H_KI, DriveConstants.H_KD);

    @Override
    public void init() {
        m_LeftFrontDrive = hardwareMap.get(DcMotorEx.class, "lfd");
        m_LeftBackDrive = hardwareMap.get(DcMotorEx.class, "lbd");
        m_RightFrontDrive = hardwareMap.get(DcMotorEx.class, "rfd");
        m_RightBackDrive = hardwareMap.get(DcMotorEx.class, "rbd");
        m_Pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pp");

        m_LeftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_LeftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_RightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_RightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Initial Directions
        m_LeftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        m_LeftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        m_RightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        m_RightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        m_MecanumDrive = new MecanumDrive(m_LeftFrontDrive, m_LeftBackDrive, m_RightFrontDrive, m_RightBackDrive);

        m_Pinpoint.initialize();
        m_Pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        m_Pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        m_Pinpoint.setOffsets(DriveConstants.ODOMETERY_Y_OFFSET, DriveConstants.ODOMETERY_X_OFFSET, DistanceUnit.INCH);
        m_Pinpoint.recalibrateIMU();
        m_Pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, m_StartingPosition.getY(DistanceUnit.INCH), -m_StartingPosition.getX(DistanceUnit.INCH), AngleUnit.DEGREES, -m_StartingPosition.getHeading(AngleUnit.DEGREES)));

        m_Localizer = new Localizer(m_MecanumDrive, DriveConstants.XY_LOCALIZER_TOLERANCE_IN, DistanceUnit.INCH, DriveConstants.H_LOCALIZER_TOLERANCE_DEGREE, AngleUnit.DEGREES, XYCoefficients, XYCoefficients, HCoefficients );
    }

    @Override
    public void init_loop() {
        m_Pinpoint.update();
        telemetry.addData("x_enc", m_Pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("y_enc", m_Pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("h_enc", m_Pinpoint.getHeading(AngleUnit.DEGREES));
    }

    protected void UpdateLocalizerParameters() {
        m_Localizer.SetYPIDCoefficients(new PIDCoefficients(DriveConstants.XY_KP, DriveConstants.XY_KI, DriveConstants.XY_KD));
        m_Localizer.SetXPIDCoefficients(new PIDCoefficients(DriveConstants.XY_KP, DriveConstants.XY_KI, DriveConstants.XY_KD));
        m_Localizer.SetHPIDCoefficients(new PIDCoefficients(DriveConstants.H_KP, DriveConstants.H_KI, DriveConstants.H_KD));
        m_Localizer.SetXYTolerance(DriveConstants.XY_LOCALIZER_TOLERANCE_IN, DistanceUnit.INCH);
        m_Localizer.SetHTolerance(DriveConstants.H_LOCALIZER_TOLERANCE_DEGREE, AngleUnit.DEGREES);
    }
}
