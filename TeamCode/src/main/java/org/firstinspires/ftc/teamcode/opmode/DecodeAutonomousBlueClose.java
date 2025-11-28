package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

//@Disabled
@Autonomous(name = "DECODE Auto Blue Close")
public class DecodeAutonomousBlueClose extends BaseDecodeAutonomous {

    @Override
    public void init() {
        m_StartingPosition = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

        m_LaunchPosition = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        m_Spike1Position = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        m_Spike2Position = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        m_Spike3Position = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        m_OpenGatePosition = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

        m_ArtifactLengthIN = 0;
        m_GateForwardDistanceIN = 0;

        super.init();
    }
}
