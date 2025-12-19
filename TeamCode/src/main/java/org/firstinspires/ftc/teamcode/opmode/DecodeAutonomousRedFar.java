package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.ControlConstants;

//@Disabled
@Autonomous(name = "DECODE Auto Red Far", preselectTeleOp = "DECODE Teleop")
public class DecodeAutonomousRedFar extends BaseDecodeAutonomous {

    @Override
    public void init() {
        m_StartingPosition = new Pose2D(DistanceUnit.INCH, 24, -62, AngleUnit.DEGREES, 0);

        m_LaunchPosition = new Pose2D(DistanceUnit.INCH, 14.5, -50, AngleUnit.DEGREES, 15);
        m_Spike1Position = new Pose2D(DistanceUnit.INCH, 29, -38.00, AngleUnit.DEGREES, 90);
        m_Spike2Position = new Pose2D(DistanceUnit.INCH, 29, -15.00, AngleUnit.DEGREES, 90);
        m_Spike3Position = new Pose2D(DistanceUnit.INCH, 29, 11.00, AngleUnit.DEGREES, 90);
        m_OpenGatePosition = new Pose2D(DistanceUnit.INCH, 53.00, -1.50, AngleUnit.DEGREES, -90);

        m_ArtifactLengthIN = 5.00;
        m_GateForwardDistanceIN = 0;
        m_launchVelocity = ControlConstants.CLOSE_LAUNCH_SPEED;

        super.init();
    }
}
