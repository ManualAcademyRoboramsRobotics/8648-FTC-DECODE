package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;


@Disabled
@TeleOp(name = "Left Right Tuner")
public class LocalizerLeftRightTuner extends BaseOpMode {
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

        telemetry.addData("x_desired", m_Localizer.GetDesiredPosition() == null ? 0 : m_Localizer.GetDesiredPosition().getX(DistanceUnit.INCH));
        telemetry.addData("y_desired", m_Localizer.GetDesiredPosition() == null ? 0 : m_Localizer.GetDesiredPosition().getY(DistanceUnit.INCH));
        telemetry.addData("h_desired", m_Localizer.GetDesiredPosition() == null ? 0 : m_Localizer.GetDesiredPosition().getHeading(AngleUnit.DEGREES));
        telemetry.addData("x_current", m_Localizer.GetCurrentPosition() == null ? 0 : m_Localizer.GetCurrentPosition().getX(DistanceUnit.INCH));
        telemetry.addData("y_current", m_Localizer.GetCurrentPosition() == null ? 0 : m_Localizer.GetCurrentPosition().getY(DistanceUnit.INCH));
        telemetry.addData("h_current", m_Localizer.GetCurrentPosition() == null ? 0 : m_Localizer.GetCurrentPosition().getHeading(AngleUnit.DEGREES));
        telemetry.addData("in_position", m_Localizer.GetCurrentPosition() == null ? 0 : m_Localizer.InPosition());
        telemetry.addData("in_bounds", m_Localizer.GetCurrentPosition() == null || m_Localizer.GetDesiredPosition() == null ? 0 : m_Localizer.InBounds());
        telemetry.addData("xin_bounds", m_Localizer.GetCurrentPosition() == null || m_Localizer.GetDesiredPosition() == null ? 0 : m_Localizer.XInBounds());
        telemetry.addData("yin_bounds", m_Localizer.GetCurrentPosition() == null || m_Localizer.GetDesiredPosition() == null ? 0 : m_Localizer.YInBounds());
        telemetry.addData("hin_bounds", m_Localizer.GetCurrentPosition() == null || m_Localizer.GetDesiredPosition() == null ? 0 : m_Localizer.HInBounds());
        telemetry.addData("localizer_state", m_Localizer.m_CurrentState);
        telemetry.addData("max_power", m_Localizer.GetMaxPower());
        telemetry.update();
    }
}
