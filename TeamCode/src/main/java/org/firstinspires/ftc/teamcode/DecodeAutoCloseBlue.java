package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Decode AutoDecodeAutoCloseBlue", preselectTeleOp = "DecodeTeleop")
public class DecodeAutoCloseBlue extends DecodeAutonomous{
    @Override
    public void init(){
        super.init();
        setLauncherVelocity(LAUNCHER_CLOSE_TARGET_VELOCITY);
        driveTimeMS = 2200;
        turnTimeMS = 1650;
        turnSpeed = -0.5;
        driveSpeed = 0.5;
    }
}
