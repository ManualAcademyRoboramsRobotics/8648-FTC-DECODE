package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;

//@Disabled
@TeleOp(name = "Drive Motor Direction Debugger")
public class DriveMotorDirectionDebugger extends BaseOpMode {

    @Override
    public void init() {
        super.init();
        telemetry.addData("INSTRUCTIONS:", "PRESS START - ALL WHEELS SHOULD ROTATE IN FORWARD DIRECTION. PRESS STOP TO STOP WHEELS");
        telemetry.addData("WARNING:", "WHEELS WILL BEGIN MOVING IMMEDIATELY AFTER HITTING START!");
    }

    @Override
    public void loop() {
        m_MecanumDrive.Drive(0.25,0,0);

        telemetry.addData("INSTRUCTIONS:", "PRESS START - ALL WHEELS SHOULD ROTATE IN FORWARD DIRECTION. PRESS STOP TO STOP WHEELS");
        telemetry.update();
    }
}
