package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drive.BaseDrive;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.PIDController;


@Autonomous (name = "Drive Motor Direction Tuner")
public class DriveMotorDirectionTuner extends BaseOpMode {
    Pose2D DesiredPose = null;

    enum pos {
        pos1,
        pos2
    }

    private pos goToPos = pos.pos1;

    @Override
    public void init() {
        DesiredPose = new Pose2D(DistanceUnit.INCH, 0, 20, AngleUnit.DEGREES, Constants.H_Desired);

        telemetry.addData("Status", "Odometer Initialized");
        }

    @Override
    public void init_loop(){
        pinpoint.update();
        telemetry.addData("x_enc", pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("y_enc", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("h_enc", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("x_offset", pinpoint.getXOffset(DistanceUnit.INCH));
        telemetry.addData("y_offset", pinpoint.getYOffset(DistanceUnit.INCH));
        telemetry.addData("Position", pinpoint.getPosition());
    }

    @Override
    public void loop() {
        pinpoint.update();

        Pose2D CurrentPose = new Pose2D(DistanceUnit.INCH, -pinpoint.getPosY(DistanceUnit.INCH), pinpoint.getPosX(DistanceUnit.INCH), AngleUnit.DEGREES, -pinpoint.getHeading(AngleUnit.DEGREES));

        localizer.setDesiredPosition(DesiredPose);
        localizer.localize(CurrentPose);

        if (localizer.inPosition()) {
            switch (goToPos) {
                case pos1:
                    DesiredPose = new Pose2D(DistanceUnit.INCH, 0, 20, AngleUnit.DEGREES, Constants.H_Desired);
                    goToPos = pos.pos2;
                    break;
                case pos2:
                    DesiredPose = new Pose2D(DistanceUnit.INCH, 0, -20, AngleUnit.DEGREES, Constants.H_Desired);
                    goToPos = pos.pos1;
                    break;
            }
        }


        telemetry.addData("x_enc", pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("y_enc", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("h_enc", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("x_desired", DesiredPose.getX(DistanceUnit.INCH));
        telemetry.addData("y_desired", DesiredPose.getY(DistanceUnit.INCH));
        telemetry.addData("h_desired", DesiredPose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", pinpoint.getPosition());
        telemetry.update();


    }
}
