package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.webserver.websockets.CommandNotImplementedException;
import org.firstinspires.ftc.teamcode.util.PIDController;

@Autonomous (name = "Decode Auto")
public class DecodeAutonomous extends DecodeControl {
    private GoBildaPinpointDriver pinpointDriver = null;
    private PIDController xPIDControl = null;
    private PIDController yPIDControl = null;
    private PIDController hPIDControl = null;

    final PIDCoefficients Coefficients = new PIDCoefficients(0.2,0,0);

    @Override
    public void init() {
        super.init();

        xPIDControl = new PIDController(Coefficients);
        yPIDControl = new PIDController(Coefficients);
        hPIDControl = new PIDController(Coefficients);

        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
        pinpointDriver.setOffsets(-3.125, -6.625, DistanceUnit.INCH); //these are tuned for 3110-0002-0001 Product Insight #1
        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpointDriver.resetPosAndIMU();
        telemetry.addData("Status", "Odometer Initialized");
        }

    @Override
    public void loop() {
        pinpointDriver.update();

        Pose2D DesiredPose = new Pose2D(DistanceUnit.INCH, 15,10, AngleUnit.DEGREES, 45);
        double y = yPIDControl.pidControl(pinpointDriver.getPosY(DistanceUnit.INCH), DesiredPose.getY(DistanceUnit.INCH));
        double x = xPIDControl.pidControl(pinpointDriver.getPosX(DistanceUnit.INCH), DesiredPose.getX(DistanceUnit.INCH));
        double h = hPIDControl.pidControl(pinpointDriver.getHeading(AngleUnit.DEGREES), DesiredPose.getHeading(AngleUnit.DEGREES));

        mecanumDrive.drive(y, x, h);

        telemetry.addData("X_PID", x);
        telemetry.addData("Y_PID", y);
        telemetry.addData("H_PID", h);
        telemetry.addData("X", pinpointDriver.getPosX(DistanceUnit.INCH));
        telemetry.addData("Y", pinpointDriver.getPosY(DistanceUnit.INCH));
        telemetry.addData("Heading", pinpointDriver.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", pinpointDriver.getPosition());
        telemetry.addData("Status", pinpointDriver.getDeviceStatus());
        telemetry.addData("Device Version Number:", pinpointDriver.getDeviceVersion());
        telemetry.addData("Heading Scalar", pinpointDriver.getYawScalar());
        telemetry.update();


    }
}
