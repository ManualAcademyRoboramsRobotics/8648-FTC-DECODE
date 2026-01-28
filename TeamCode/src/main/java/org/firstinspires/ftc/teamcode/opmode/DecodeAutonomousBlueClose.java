package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.ControlConstants;

//@Disabled
@Autonomous(name = "DECODE Auto Blue Close", preselectTeleOp = "DECODE Teleop")
public class DecodeAutonomousBlueClose extends BaseDecodeAutonomous {

    @Override
    public void init() {
        m_StartingPosition = new Pose2D(DistanceUnit.INCH, -AutoConstants.CLOSE_START_X, AutoConstants.CLOSE_START_Y, AngleUnit.DEGREES, -AutoConstants.CLOSE_START_H);

        m_LaunchPosition = new Pose2D(DistanceUnit.INCH, -AutoConstants.CLOSE_LAUNCH_X, AutoConstants.CLOSE_LAUNCH_Y, AngleUnit.DEGREES, -AutoConstants.CLOSE_LAUNCH_H);
        m_Spike1Position = new Pose2D(DistanceUnit.INCH, -AutoConstants.SPIKE_X, AutoConstants.SPIKE_1_Y, AngleUnit.DEGREES, -90);
        m_Spike2Position = new Pose2D(DistanceUnit.INCH, -AutoConstants.SPIKE_X, AutoConstants.SPIKE_2_Y, AngleUnit.DEGREES, -90);
        m_Spike3Position = new Pose2D(DistanceUnit.INCH, -AutoConstants.SPIKE_X, AutoConstants.SPIKE_3_Y, AngleUnit.DEGREES, -90);
        m_OpenGatePosition = new Pose2D(DistanceUnit.INCH, -53.00, -1.50, AngleUnit.DEGREES, -90);

        m_ArtifactLengthIN = -AutoConstants.ARTIFACT_LENGTH_IN;
        m_GateForwardDistanceIN = 0;
        m_launchVelocity = ControlConstants.CLOSE_LAUNCH_SPEED;
        ControlConstants.CURRENT_ALLIANCE = ControlConstants.Alliance.BLUE;

        super.init();
    }
}
