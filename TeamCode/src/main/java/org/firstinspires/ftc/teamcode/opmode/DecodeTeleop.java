package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.ControlConstants;
import org.firstinspires.ftc.teamcode.control.DecodeControl;

@TeleOp(name = "DECODE Teleop")
//@Disabled
public class DecodeTeleop extends BaseOpMode {
    private DecodeControl m_Controls;

    private boolean AutoAiming = false;

    double requestedVelocity = 2100;

    double launcherTarget = requestedVelocity; //These variables allow
    double launcherMin = requestedVelocity - 100;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        super.init();
        m_Controls = new DecodeControl(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        //Auto Aiming Logic
        if(AutoAiming)
        {
            m_Pinpoint.update();
            Pose2D CurrentPose = new Pose2D(DistanceUnit.INCH, -m_Pinpoint.getPosY(DistanceUnit.INCH), m_Pinpoint.getPosX(DistanceUnit.INCH), AngleUnit.DEGREES, -m_Pinpoint.getHeading(AngleUnit.DEGREES));
            double DesiredHeading = 0;
            if (ControlConstants.CURRENT_ALLIANCE == ControlConstants.Alliance.BLUE)
            {
                DesiredHeading = Math.atan2((ControlConstants.BLUE_GOAL_X-CurrentPose.getX(DistanceUnit.INCH)),(ControlConstants.BLUE_GOAL_Y-CurrentPose.getY(DistanceUnit.INCH)));
            } else {
                DesiredHeading = Math.atan2((ControlConstants.RED_GOAL_X-CurrentPose.getX(DistanceUnit.INCH)),(ControlConstants.RED_GOAL_Y-CurrentPose.getY(DistanceUnit.INCH)));
            }
            m_Localizer.SetDesiredPosition(new Pose2D(DistanceUnit.INCH, CurrentPose.getX(DistanceUnit.INCH), CurrentPose.getY(DistanceUnit.INCH), AngleUnit.RADIANS, DesiredHeading));

            m_Localizer.Localize(CurrentPose);
        } else {
            m_MecanumDrive.Drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        if (Math.abs(gamepad1.left_stick_y) > ControlConstants.STICK_THRESHOLD || Math.abs(gamepad1.left_stick_x) > ControlConstants.STICK_THRESHOLD || Math.abs(gamepad1.right_stick_x) > ControlConstants.STICK_THRESHOLD)
        {
            AutoAiming = false;
        }

        if (Math.abs(gamepad1.left_trigger) > 0 || Math.abs(gamepad1.right_trigger) > 0)
        {
            AutoAiming = true;
        }

        if (gamepad1.y) {
            m_Controls.launcherSpinUp();
        }

        if (gamepad1.b) {
            m_Controls.launcherStop();
        }

        if (gamepad1.dpadDownWasPressed()) {
            m_Controls.diverterDirectionToggle();
        }

        if (gamepad1.aWasPressed()){
            m_Controls.intakeStateToggle();
        }

        if (gamepad1.dpadUpWasPressed()) {
            m_Controls.launcherVelocityToggle();
            m_Controls.setLauncherVelocity(requestedVelocity);
        }

        if (gamepad1.dpadLeftWasPressed()) {
            requestedVelocity -= 100;
            m_Controls.setLauncherVelocity(requestedVelocity);

        }

        if (gamepad1.dpadRightWasPressed()) {
            requestedVelocity += 100;
            m_Controls.setLauncherVelocity(requestedVelocity);
        }

        m_Controls.launchLeft(gamepad1.leftBumperWasPressed());
        m_Controls.launchRight(gamepad1.rightBumperWasPressed());

        /*
         * Show the state and motor powers
         */
        telemetry.addData("Left State", m_Controls.leftLauncherState);
        telemetry.addData("Right State", m_Controls.rightLauncherState);
        telemetry.addData("launch distance", m_Controls.launcherDistance);
        telemetry.addData("launcher velocity", m_Controls.launcherVelocity);
        telemetry.addData("Left Launcher Velocity", Math.abs(m_Controls.GetLeftLauncherVelocity()));
        telemetry.addData("Right Launcher Velocity", Math.abs(m_Controls.GetRightLauncherVelocity()));
        telemetry.addData("Diverter Direction", m_Controls.diverterDirection);
        telemetry.update();

    }
}