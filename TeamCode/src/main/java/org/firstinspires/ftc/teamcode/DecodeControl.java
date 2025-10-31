package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.TaskCallbackTimer;

import java.util.Timer;

public abstract class DecodeControl extends OpMode {

    //////////////////////////////////////////////////////////////
    // Constants
    //////////////////////////////////////////////////////////////
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    final long FEED_TIME_MS = 800; //The feeder servos run this long when a shot is requested.

    final double  LAUNCHER_CLOSE_TARGET_VELOCITY = 1500;
    final double LAUNCHER_FAR_TARGET_VELOCITY = 1700;
    final double ALLOWED_VELOCITY_DIVERSION = 100;

//    final double LEFT_POSITION = 0.348;
    final double LEFT_POSITION = 0.555;
//    final double RIGHT_POSITION = 0.310;
    final double RIGHT_POSITION = 0.08;


    //////////////////////////////////////////////////////////////
    // Enums
    //////////////////////////////////////////////////////////////
    protected enum LaunchState {
        OFF,
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    protected enum IntakeState {
        ON,
        OFF;
    }

    protected enum DiverterDirection {
        LEFT,
        RIGHT;
    }

    protected enum LauncherDistance {
        CLOSE,
        FAR;
    }

    //////////////////////////////////////////////////////////////
    // Member Variables
    //////////////////////////////////////////////////////////////

    // Control Components
    protected DcMotorEx leftFrontDrive = null;
    protected DcMotorEx leftBackDrive = null;
    protected DcMotorEx rightFrontDrive = null;
    protected DcMotorEx rightBackDrive = null;
    protected DcMotorEx intake = null;
    protected DcMotorEx leftLauncher = null;
    protected DcMotorEx rightLauncher = null;
    protected CRServo leftFeeder = null;
    protected CRServo rightFeeder = null;
    protected Servo diverter = null;


    // Control Objects
    protected MecanumDrive mecanumDrive = null;
    protected Timer feederTimer = null;
    protected PIDFCoefficients feederPIDFCoefficients = null;


    // State Variables
    protected LaunchState leftLauncherState = LaunchState.OFF;
    protected LaunchState rightLauncherState = LaunchState.OFF;
    protected DiverterDirection diverterDirection = DiverterDirection.RIGHT;
    protected IntakeState intakeState = IntakeState.OFF;
    protected LauncherDistance launcherDistance = LauncherDistance.CLOSE;
    protected double launcherVelocity = LAUNCHER_CLOSE_TARGET_VELOCITY;


    @Override
    public void init() {
        feederPIDFCoefficients = new PIDFCoefficients(300, 0, 0, 10);

        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "lfd");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "lbd");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rfd");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rbd");
        leftLauncher = hardwareMap.get(DcMotorEx.class, "ll");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rl");
        intake = hardwareMap.get(DcMotorEx.class, "i");
        leftFeeder = hardwareMap.get(CRServo.class, "lf");
        rightFeeder = hardwareMap.get(CRServo.class, "rf");
        diverter = hardwareMap.get(Servo.class, "d");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Initial Directions
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, feederPIDFCoefficients);
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, feederPIDFCoefficients);

        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        diverter.setPosition(RIGHT_POSITION);

        // Set up timer tasks so we don't forget to turn off the feeder
        feederTimer = new Timer();

        // Initialize mecanum drive
        mecanumDrive = new MecanumDrive(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive);
    }

    protected void launcherSpinUp() {
        leftLauncherState = LaunchState.IDLE;
        rightLauncherState = LaunchState.IDLE;
        leftLauncher.setVelocity(launcherVelocity);
        rightLauncher.setVelocity(launcherVelocity);
    }

    protected void launcherStop() {
        leftLauncherState = LaunchState.OFF;
        rightLauncherState = LaunchState.OFF;
        leftLauncher.setVelocity(STOP_SPEED);
        rightLauncher.setVelocity(STOP_SPEED);
    }

    protected void diverterLeft() {
        diverterDirection = DiverterDirection.LEFT;
        diverter.setPosition(LEFT_POSITION);
    }

    protected void diverterRight() {
        diverterDirection = DiverterDirection.RIGHT;
        diverter.setPosition(RIGHT_POSITION);
    }

    protected void diverterDirectionToggle() {
        switch (diverterDirection){
            case LEFT:
                diverterRight();
                break;
            case RIGHT:
                diverterLeft();
                break;
        }
    }

    protected void intakeOn() {
        intakeState = IntakeState.ON;
        intake.setPower(1);
    }

    protected void intakeOff(){
        intakeState = IntakeState.OFF;
        intake.setPower(0);
    }

    protected void intakeStateToggle() {
        switch (intakeState){
            case ON:
                intakeOff();
                break;
            case OFF:
                intakeOn();
                break;
        }
    }

    protected void launcherVelocityToggle() {
        switch (launcherDistance) {
            case CLOSE:
                launcherDistance = LauncherDistance.FAR;
                setLauncherVelocity(LAUNCHER_FAR_TARGET_VELOCITY);
                break;
            case FAR:
                launcherDistance = LauncherDistance.CLOSE;
                setLauncherVelocity(LAUNCHER_CLOSE_TARGET_VELOCITY);
                break;
        }
    }

    protected void setLauncherVelocity(double velocity) {
        launcherVelocity = velocity;
        if (leftLauncherState != LaunchState.OFF || rightLauncherState != LaunchState.OFF )
        {
            leftLauncher.setVelocity(launcherVelocity);
            rightLauncher.setVelocity(launcherVelocity);
        }
    }

    private void leftLauncherStartFeed() {
        leftFeeder.setPower(FULL_SPEED);
        leftLauncherState = LaunchState.LAUNCHING;
        feederTimer.schedule(new TaskCallbackTimer(this::leftLauncherFinishFeed), FEED_TIME_MS);
    }

    private void leftLauncherFinishFeed() {
        leftFeeder.setPower(STOP_SPEED);
        leftLauncherState = LaunchState.IDLE;
    }

    private void rightLauncherStartFeed() {
        rightFeeder.setPower(FULL_SPEED);
        rightLauncherState = LaunchState.LAUNCHING;
        feederTimer.schedule(new TaskCallbackTimer(this::rightLauncherFinishFeed), FEED_TIME_MS);
    }

    private void rightLauncherFinishFeed() {
        rightFeeder.setPower(STOP_SPEED);
        rightLauncherState = LaunchState.IDLE;
    }

    void launchLeft(boolean shotRequested) {
        switch (leftLauncherState) {
            case OFF:
            case IDLE:
                if (shotRequested) {
                    launcherSpinUp();
                    leftLauncherState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcherSpinUp();
                if (Math.abs(leftLauncher.getVelocity()) > (launcherVelocity - ALLOWED_VELOCITY_DIVERSION)) {
                    leftLauncherState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftLauncherStartFeed();
                break;
        }
    }

    void launchRight(boolean shotRequested) {
        switch (rightLauncherState) {
            case OFF:
            case IDLE:
                if (shotRequested) {
                    launcherSpinUp();
                    rightLauncherState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                if (Math.abs(rightLauncher.getVelocity()) > (launcherVelocity - ALLOWED_VELOCITY_DIVERSION)) {
                    rightLauncherState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                rightLauncherStartFeed();
                break;
        }
    }
}
