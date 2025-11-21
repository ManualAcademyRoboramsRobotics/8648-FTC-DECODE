package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Localizer {

    public enum States{
        IDLE,
        DRIVING,
        POSITIONED
    }
    private final PIDController xPIDControl;
    private final PIDController yPIDControl;
    private final PIDController hPIDControl;

    private final MecanumDrive Drive;

    private Pose2D CurrentPosition;
    private Pose2D DesiredPosition;
    private States CurrentState;

    private static double XYTolerance = 12;
    private static double YawTolerance = 0.0349066;

    public Localizer(MecanumDrive drive, double xyTolerance, double yawTolerance, PIDCoefficients xCoefficients, PIDCoefficients yCoefficients, PIDCoefficients hCoefficients) {
        xPIDControl = new PIDController(xCoefficients);
        yPIDControl = new PIDController(yCoefficients);
        hPIDControl = new PIDController(hCoefficients);
        Drive = drive;
        XYTolerance = xyTolerance;
        YawTolerance = yawTolerance;

    }
    public void localize(Pose2D position)
    {
        CurrentPosition = position;
        if (CurrentState != States.IDLE)
        {
            drive();
        }
    }

    public void setDesiredPosition(Pose2D position){
        DesiredPosition = position;
        CurrentState = States.DRIVING;
    }

    public void setIdle(){
        CurrentState = States.IDLE;
    }
    private void drive() {
        double y = yPIDControl.pidControl(CurrentPosition.getY(DistanceUnit.INCH), DesiredPosition.getY(DistanceUnit.INCH));
        double x = xPIDControl.pidControl(CurrentPosition.getX(DistanceUnit.INCH), DesiredPosition.getX(DistanceUnit.INCH));
        double h = hPIDControl.pidControl(CurrentPosition.getHeading(AngleUnit.DEGREES), DesiredPosition.getHeading(AngleUnit.DEGREES));

        double cosine = Math.cos(CurrentPosition.getHeading(AngleUnit.RADIANS));
        double sine = Math.sin(CurrentPosition.getHeading(AngleUnit.RADIANS));

        double yOutput = (y * cosine) - (x * sine);
        double xOutput = (y * sine) + (x * cosine);

        Drive.drive(yOutput, xOutput, h);

        if (inBounds()){
            CurrentState = States.POSITIONED;
        }
    }

    public boolean inPosition(){
        return CurrentState == States.POSITIONED;
    }

    private boolean inBounds() {
        boolean xInBounds = CurrentPosition.getX(DistanceUnit.MM) > (DesiredPosition.getX(DistanceUnit.MM) - XYTolerance) &&
                CurrentPosition.getX(DistanceUnit.MM) < (DesiredPosition.getX(DistanceUnit.MM) + XYTolerance);
        boolean yInBounds = CurrentPosition.getY(DistanceUnit.MM) > (DesiredPosition.getY(DistanceUnit.MM) - XYTolerance) &&
                CurrentPosition.getY(DistanceUnit.MM) < (DesiredPosition.getY(DistanceUnit.MM) + XYTolerance);
        boolean hInBounds = CurrentPosition.getHeading(AngleUnit.RADIANS) > (DesiredPosition.getHeading(AngleUnit.RADIANS) - YawTolerance) &&
                CurrentPosition.getHeading(AngleUnit.RADIANS) < (DesiredPosition.getHeading(AngleUnit.RADIANS) + YawTolerance);

        return xInBounds && yInBounds && hInBounds;
    }
}
