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


@Autonomous (name = "PID Tuning")
public class DecodeAutonomousLocalizer extends OpMode {
    //http://192.168.43.1:8080/dash
    private FtcDashboard dashboard = null;
    protected DcMotorEx leftFrontDrive = null;
    protected DcMotorEx leftBackDrive = null;
    protected DcMotorEx rightFrontDrive = null;
    protected DcMotorEx rightBackDrive = null;
    protected MecanumDrive mecanumDrive = null;
    private GoBildaPinpointDriver pinpoint = null;
    private Localizer localizer = null;
    private PIDController xPIDControl = null;
    private PIDController yPIDControl = null;
    private PIDController hPIDControl = null;

//    final PIDCoefficients YCoefficients = new PIDCoefficients(0.0002,0,0);
    final PIDCoefficients XYCoefficients = new PIDCoefficients(Constants.Kp_Lateral, Constants.Ki_Lateral, Constants.Kd_Lateral);
//    final PIDCoefficients HCoefficients = new PIDCoefficients(0.01,0,0);
    final PIDCoefficients HCoefficients = new PIDCoefficients(Constants.Kp_Heading, Constants.Ki_Heading, Constants.Kd_Heading);

    Pose2D DesiredPose = null;

    enum pos {
        pos1,
        pos2,
        pos3,
        pos4
    }

    private pos goToPos = pos.pos1;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
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
        pinpoint.setOffsets(Constants.X_Offset_in, Constants.Y_Offset_in, DistanceUnit.INCH);
        //pinpoint.setOffsets(-4.75,1, DistanceUnit.INCH);
        pinpoint.resetPosAndIMU();

        xPIDControl = new PIDController(XYCoefficients);
        yPIDControl = new PIDController(XYCoefficients);
        hPIDControl = new PIDController(HCoefficients);

        localizer = new Localizer(mecanumDrive, 12, 0.0349066, XYCoefficients, XYCoefficients, HCoefficients );

        DesiredPose = new Pose2D(DistanceUnit.INCH, 0, 20, AngleUnit.DEGREES, Constants.H_Desired);

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

        Pose2D CurrentPose = new Pose2D(DistanceUnit.INCH, -pinpoint.getPosY(DistanceUnit.INCH), pinpoint.getPosX(DistanceUnit.INCH), AngleUnit.DEGREES, -pinpoint.getHeading(AngleUnit.DEGREES));

        localizer.setDesiredPosition(DesiredPose);
        localizer.localize(CurrentPose);

        if (localizer.inPosition()) {
            switch (goToPos) {
                case pos1:
                    DesiredPose = new Pose2D(DistanceUnit.INCH, 0, 20, AngleUnit.DEGREES, Constants.H_Desired);
                    goToPos = pos.pos2;
                    break;
                case pos2:
                    DesiredPose = new Pose2D(DistanceUnit.INCH, 20, 0, AngleUnit.DEGREES, Constants.H_Desired);
                    goToPos = pos.pos3;
                    break;
                case pos3:
                    DesiredPose = new Pose2D(DistanceUnit.INCH, 0, -20, AngleUnit.DEGREES, Constants.H_Desired);
                    goToPos = pos.pos4;
                    break;
                case pos4:
                    DesiredPose = new Pose2D(DistanceUnit.INCH, -20, 0, AngleUnit.DEGREES, Constants.H_Desired);
                    goToPos = pos.pos1;
                    break;
            }
        }


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
