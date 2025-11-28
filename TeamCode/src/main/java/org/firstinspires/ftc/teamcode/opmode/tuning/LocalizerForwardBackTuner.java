package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;


//@Disabled
@TeleOp(name = "Forward Back Tuner")
public class LocalizerForwardBackTuner extends BaseOpMode {
    Pose2D DesiredPose;

    enum pos {
        pos1,
        pos2
    }

    private pos goToPos;

    final Pose2D Position1 = new Pose2D(DistanceUnit.INCH, 24, 0, AngleUnit.DEGREES, 0);
    final Pose2D Position2 = new Pose2D(DistanceUnit.INCH, -24, 0, AngleUnit.DEGREES, 0);

    @Override
    public void init() {
        super.init();
        goToPos = pos.pos1;
        DesiredPose = Position1;
     }

    @Override
    public void loop() {
        m_Pinpoint.update();

        Pose2D CurrentPose = new Pose2D(DistanceUnit.INCH, -m_Pinpoint.getPosY(DistanceUnit.INCH), m_Pinpoint.getPosX(DistanceUnit.INCH), AngleUnit.DEGREES, -m_Pinpoint.getHeading(AngleUnit.DEGREES));

        m_Localizer.SetDesiredPosition(DesiredPose);
        UpdateLocalizerParameters();
        m_Localizer.Localize(CurrentPose);

        if (m_Localizer.InPosition()) {
            switch (goToPos) {
                case pos1:
                    DesiredPose = Position1;
                    goToPos = pos.pos2;
                    break;
                case pos2:
                    DesiredPose = Position2;
                    goToPos = pos.pos1;
                    break;
            }
        }

        telemetry.addData("x_enc", m_Pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("y_enc", m_Pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("h_enc", m_Pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("x_desired", DesiredPose.getX(DistanceUnit.INCH));
        telemetry.addData("y_desired", DesiredPose.getY(DistanceUnit.INCH));
        telemetry.addData("h_desired", DesiredPose.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }
}
