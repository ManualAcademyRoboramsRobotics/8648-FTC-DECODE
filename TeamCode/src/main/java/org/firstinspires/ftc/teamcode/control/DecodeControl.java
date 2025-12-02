package org.firstinspires.ftc.teamcode.control;

import android.service.controls.Control;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.ControlConstants;
import org.firstinspires.ftc.teamcode.util.TaskCallbackTimer;

import java.util.Timer;

public class DecodeControl {

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
    public enum LaunchState {
        OFF,
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    public enum IntakeState {
        ON,
        OFF;
    }

    public enum DiverterDirection {
        LEFT,
        RIGHT;
    }

    public enum LauncherDistance {
        CLOSE,
        FAR;
    }

    //////////////////////////////////////////////////////////////
    // Member Variables
    //////////////////////////////////////////////////////////////

    // Control Components
    final DcMotorEx intake, leftLauncher, rightLauncher;
    final CRServo leftFeeder, rightFeeder;
    final Servo diverter;

    // Control Objects
    private Timer feederTimer = null;
    final PIDFCoefficients launcherfeederPIDFCoefficients = new PIDFCoefficients(ControlConstants.LAUNCHER_KP, ControlConstants.LAUNCHER_KI, ControlConstants.LAUNCHER_KD, ControlConstants.LAUNCHER_KF);

    // State Variables
    public LaunchState leftLauncherState = LaunchState.OFF, rightLauncherState = LaunchState.OFF;
    public DiverterDirection diverterDirection = DiverterDirection.RIGHT;
    public IntakeState intakeState = IntakeState.OFF;
    public LauncherDistance launcherDistance = LauncherDistance.CLOSE;
    public double launcherVelocity = LAUNCHER_CLOSE_TARGET_VELOCITY;

    public DecodeControl(HardwareMap hardwareMap) {
        leftLauncher = hardwareMap.get(DcMotorEx.class, "ll");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rl");
        intake = hardwareMap.get(DcMotorEx.class, "i");
        leftFeeder = hardwareMap.get(CRServo.class, "lf");
        rightFeeder = hardwareMap.get(CRServo.class, "rf");
        diverter = hardwareMap.get(Servo.class, "d");

        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Initial Directions
        leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        /////////////////////////////////////////////////////////////////////////////////////////////////
        // 9788!!!!!
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // 8648!!!!!
        //intake.setDirection(DcMotorSimple.Direction.REVERSE);
        /////////////////////////////////////////////////////////////////////////////////////////////////
        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherfeederPIDFCoefficients);
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherfeederPIDFCoefficients);

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        diverter.setPosition(RIGHT_POSITION);

        // Set up timer tasks so we don't forget to turn off the feeder
        feederTimer = new Timer();
    }

    public void launcherSpinUp() {
        leftLauncherState = LaunchState.IDLE;
        rightLauncherState = LaunchState.IDLE;
        leftLauncher.setVelocity(launcherVelocity);
        rightLauncher.setVelocity(launcherVelocity);
    }

    public void launcherStop() {
        leftLauncherState = LaunchState.OFF;
        rightLauncherState = LaunchState.OFF;
        leftLauncher.setVelocity(STOP_SPEED);
        rightLauncher.setVelocity(STOP_SPEED);
    }

    public void diverterLeft() {
        diverterDirection = DiverterDirection.LEFT;
        diverter.setPosition(LEFT_POSITION);
    }

    public void diverterRight() {
        diverterDirection = DiverterDirection.RIGHT;
        diverter.setPosition(RIGHT_POSITION);
    }

    public void diverterDirectionToggle() {
        switch (diverterDirection){
            case LEFT:
                diverterRight();
                break;
            case RIGHT:
                diverterLeft();
                break;
        }
    }

    public void intakeOn() {
        intakeState = IntakeState.ON;
        intake.setPower(1);
    }

    public void intakeOff(){
        intakeState = IntakeState.OFF;
        intake.setPower(0);
    }

    public void intakeStateToggle() {
        switch (intakeState){
            case ON:
                intakeOff();
                break;
            case OFF:
                intakeOn();
                break;
        }
    }

    public void launcherVelocityToggle() {
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

    public void setLauncherVelocity(double velocity) {
        launcherVelocity = velocity;
        if (leftLauncherState != LaunchState.OFF || rightLauncherState != LaunchState.OFF )
        {
            leftLauncher.setVelocity(launcherVelocity);
            rightLauncher.setVelocity(launcherVelocity);
        }
    }

    public double GetLeftLauncherVelocity(){
        return leftLauncher.getVelocity();
    }

    public double GetRightLauncherVelocity(){
        return rightLauncher.getVelocity();
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

    public void launchLeft(boolean shotRequested) {
        switch (leftLauncherState) {
            case OFF:
            case IDLE:
                if (shotRequested) {
                    launcherSpinUp();
                    leftLauncherState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                if (Math.abs(leftLauncher.getVelocity()) > (launcherVelocity - ALLOWED_VELOCITY_DIVERSION)) {
                    leftLauncherState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftLauncherStartFeed();
                break;
        }
    }

    public void launchRight(boolean shotRequested) {
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
