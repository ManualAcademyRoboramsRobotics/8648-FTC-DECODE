package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class DecodeAutonomous extends DecodeControl {
    private ElapsedTime timer = null;
    private enum AutoState {
        IDLE,
        DRIVE,
        TURN,
        SHOOT1,
        SHOOT2,
        SHOOT3,
        COMPLETE;
    }
    protected long driveTimeMS = 0;
    protected long turnTimeMS = 0;
    protected double driveSpeed = 0;
    protected double turnSpeed = 0;
    private final long shootTime = 7250;
    private boolean needToShoot = true;

    private AutoState State = AutoState.IDLE;
//    private FtcDashboard dashboard = null;
//    private GoBildaPinpointDriver pinpointDriver = null;
//    private PIDController xPIDControl = null;
//    private PIDController yPIDControl = null;
//    private PIDController hPIDControl = null;
//    private PIDCoefficients hCoefficents = new PIDCoefficients(Constants.Kp_Heading, Constants.Ki_Heading, Constants.Kd_Heading);
//    private PIDCoefficients lCoefficents = new PIDCoefficients(Constants.Kp_Lateral, Constants.Ki_Lateral, Constants.Kd_Lateral);
//
//    //final PIDCoefficients Coefficients = new PIDCoefficients(0.2,0,0);

    @Override
    public void init() {
        super.init();
        timer = new ElapsedTime();
//        dashboard = FtcDashboard.getInstance();
//
//        xPIDControl = new PIDController(lCoefficents);
//        yPIDControl = new PIDController(lCoefficents);
//        hPIDControl = new PIDController(hCoefficents);
//
//        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
//        pinpointDriver.setOffsets(-3.125, -6.625, DistanceUnit.INCH); //these are tuned for 3110-0002-0001 Product Insight #1
//        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        pinpointDriver.resetPosAndIMU();
//        telemetry.addData("Status", "Odometer Initialized");


    }

    @Override
    public void loop() {

        switch (State){
            case IDLE:
                State = AutoState.DRIVE;
                launcherSpinUp();
                timer.reset();
                break;
            case DRIVE:
                mecanumDrive.drive(driveSpeed,0.0,0.0);
                if (timer.time(TimeUnit.MILLISECONDS) > driveTimeMS)
                {
                    mecanumDrive.drive(0,0,0);
                    timer.reset();
                    State = AutoState.TURN;
                }
                break;
            case TURN:
                mecanumDrive.drive(0.0,0.0,turnSpeed);
                if (timer.time(TimeUnit.MILLISECONDS) > turnTimeMS)
                {
                    mecanumDrive.drive(0,0,0);
                    timer.reset();
                    needToShoot = true;
                    State = AutoState.SHOOT1;
                }
                break;
            case SHOOT1:
                launchLeft(needToShoot);
                needToShoot = false;

                if (timer.time(TimeUnit.MILLISECONDS) > shootTime)
                {
                    needToShoot = true;
                    timer.reset();
                    State = AutoState.SHOOT2;
                }
                break;
            case SHOOT2:
                launchRight(needToShoot);
                needToShoot = false;

                if (timer.time(TimeUnit.MILLISECONDS) > shootTime)
                {
                    needToShoot = true;
                    timer.reset();
                    State = AutoState.SHOOT3;
                }
                break;
            case SHOOT3:
                launchLeft(needToShoot);
                needToShoot = false;

                if (timer.time(TimeUnit.MILLISECONDS) > shootTime)
                {
                    State = AutoState.COMPLETE;
                }
                break;
            case COMPLETE:
                //launcherStop();
                break;
        }

        telemetry.addData("State", State);
        telemetry.update();

//        pinpointDriver.update();
//
//        Pose2D DesiredPose = new Pose2D(DistanceUnit.INCH, 15,10, AngleUnit.DEGREES, 45);
//        double y = yPIDControl.pidControl(pinpointDriver.getPosY(DistanceUnit.INCH), DesiredPose.getY(DistanceUnit.INCH));
//        double x = xPIDControl.pidControl(pinpointDriver.getPosX(DistanceUnit.INCH), DesiredPose.getX(DistanceUnit.INCH));
//        double h = hPIDControl.pidControl(pinpointDriver.getHeading(AngleUnit.DEGREES), DesiredPose.getHeading(AngleUnit.DEGREES));
//
//        mecanumDrive.drive(y, x, h);
//
//        telemetry.addData("X_PID", x);
//        telemetry.addData("Y_PID", y);
//        telemetry.addData("H_PID", h);
//        telemetry.addData("X", pinpointDriver.getPosX(DistanceUnit.INCH));
//        telemetry.addData("Y", pinpointDriver.getPosY(DistanceUnit.INCH));
//        telemetry.addData("Heading", pinpointDriver.getHeading(AngleUnit.DEGREES));
//        telemetry.addData("Position", pinpointDriver.getPosition());
//        telemetry.addData("Status", pinpointDriver.getDeviceStatus());
//        telemetry.addData("Device Version Number:", pinpointDriver.getDeviceVersion());
//        telemetry.addData("Heading Scalar", pinpointDriver.getYawScalar());
//        telemetry.update();


    }
}
