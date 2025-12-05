package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;


@Disabled
@TeleOp(name = "Localizer Move Tuner")
public class LocalizerMoveTuner extends BaseOpMode {
    @Override
    public void init() {
        super.init();
        m_Localizer.SetDesiredPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

    }

    @Override
    public void loop() {
        m_Pinpoint.update();
        Pose2D CurrentPose = new Pose2D(DistanceUnit.INCH, -m_Pinpoint.getPosY(DistanceUnit.INCH), m_Pinpoint.getPosX(DistanceUnit.INCH), AngleUnit.DEGREES, -m_Pinpoint.getHeading(AngleUnit.DEGREES));

        UpdateLocalizerParameters();
        m_Localizer.Localize(CurrentPose);

        if (gamepad1.dpadRightWasPressed()){
            m_Localizer.MoveX(5, DistanceUnit.INCH);
        }
        if (gamepad1.dpadLeftWasPressed()){
            m_Localizer.MoveX(-5, DistanceUnit.INCH);
        }
        if (gamepad1.dpadUpWasPressed()){
            m_Localizer.MoveY(5, DistanceUnit.INCH);
        }
        if (gamepad1.dpadDownWasPressed()){
            m_Localizer.MoveY(-5, DistanceUnit.INCH);
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
