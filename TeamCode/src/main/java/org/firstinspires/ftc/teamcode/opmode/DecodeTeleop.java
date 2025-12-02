package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.DecodeControl;

@TeleOp(name = "DECODE Teleop")
//@Disabled
public class DecodeTeleop extends BaseOpMode {
    private DecodeControl m_Controls;

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
        m_MecanumDrive.Drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

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