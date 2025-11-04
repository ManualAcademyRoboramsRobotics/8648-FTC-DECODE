package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Decode AutoDecodeAutoFarBlue", preselectTeleOp = "DecodeTeleop")
public class DecodeAutoFarBlue extends DecodeAutonomous{
    @Override
    public void init(){
        super.init();
        setLauncherVelocity(LAUNCHER_FAR_TARGET_VELOCITY);
        driveTimeMS = 100;
        turnTimeMS = 190;
        turnSpeed = -0.5;
        driveSpeed = 0.5;
    }
}
