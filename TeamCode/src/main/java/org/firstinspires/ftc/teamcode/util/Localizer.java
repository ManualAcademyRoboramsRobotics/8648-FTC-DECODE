package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

import com.acmerobotics.dashboard.config.Config;

public class Localizer {

    public enum States{
        IDLE,
        DRIVING,
        POSITIONED
    }
    private final PIDController m_XPIDControl, m_YPIDControl, m_HPIDControl;

    private final MecanumDrive m_Drive;

    private Pose2D m_CurrentPosition, m_DesiredPosition;

    private States m_CurrentState;

    private double m_XYToleranceMM, m_HToleranceRadian;
    private double m_MaxPower;

    public Localizer(MecanumDrive drive, double xyTolerance, DistanceUnit xyToleranceUnit, double hTolerance, AngleUnit hToleranceUnit, PIDCoefficients xCoefficients, PIDCoefficients yCoefficients, PIDCoefficients hCoefficients) {
        m_XPIDControl = new PIDController(xCoefficients);
        m_YPIDControl = new PIDController(yCoefficients);
        m_HPIDControl = new PIDController(hCoefficients);
        m_Drive = drive;
        m_MaxPower = 1;
        m_CurrentState = States.IDLE;
        m_XYToleranceMM = DistanceUnit.MM.fromUnit(xyToleranceUnit, xyTolerance) ;
        m_HToleranceRadian = AngleUnit.RADIANS.fromUnit(hToleranceUnit, hTolerance);
    }

    public void Localize(Pose2D position)
    {
        m_CurrentPosition = position;
        if (m_CurrentState != States.IDLE)
        {
            Drive();
        }
    }

    public void SetDesiredPosition(Pose2D position){
        m_DesiredPosition = position;
        m_CurrentState = States.DRIVING;
    }

    public void SetXPIDCoefficients(PIDCoefficients coefficients) {
        m_XPIDControl.SetCoefficients(coefficients);
    }

    public void SetYPIDCoefficients(PIDCoefficients coefficients) {
        m_YPIDControl.SetCoefficients(coefficients);
    }

    public void SetHPIDCoefficients(PIDCoefficients coefficients) {
        m_HPIDControl.SetCoefficients(coefficients);
    }

    public void SetXYTolerance(double tolerance, DistanceUnit unit) {
        m_XYToleranceMM = DistanceUnit.MM.fromUnit(unit, tolerance);
    }

    public void SetHTolerance(double tolerance, AngleUnit unit) {
        m_HToleranceRadian = AngleUnit.RADIANS.fromUnit(unit, tolerance);
    }

    public void SetIdle() {
        m_CurrentState = States.IDLE;
    }

    public void SetMaxPower(double maxPower) {
        m_MaxPower = maxPower;
    }

    public Pose2D GetDesiredPosition() {
        return m_DesiredPosition;
    }

    public void MoveY(double distance, DistanceUnit unit) {
        SetDesiredPosition(new Pose2D(unit, m_DesiredPosition.getX(unit), m_DesiredPosition.getY(unit) + distance, AngleUnit.RADIANS, m_DesiredPosition.getHeading(AngleUnit.RADIANS)));
    }

    public void MoveX(double distance, DistanceUnit unit) {
        SetDesiredPosition(new Pose2D(unit, m_DesiredPosition.getX(unit) + distance, m_DesiredPosition.getY(unit), AngleUnit.RADIANS, m_DesiredPosition.getHeading(AngleUnit.RADIANS)));
    }

    public void Turn(double angle, AngleUnit unit) {
        SetDesiredPosition(new Pose2D(DistanceUnit.MM, m_DesiredPosition.getX(DistanceUnit.MM), m_DesiredPosition.getY(DistanceUnit.MM), unit, m_DesiredPosition.getHeading(unit) + angle));
    }

    private void Drive() {
        double y = m_YPIDControl.pidControl(m_CurrentPosition.getY(DistanceUnit.INCH), m_DesiredPosition.getY(DistanceUnit.INCH));
        double x = m_XPIDControl.pidControl(m_CurrentPosition.getX(DistanceUnit.INCH), m_DesiredPosition.getX(DistanceUnit.INCH));
        double h = m_HPIDControl.pidControl(m_CurrentPosition.getHeading(AngleUnit.DEGREES), m_DesiredPosition.getHeading(AngleUnit.DEGREES));

        double cosine = Math.cos(m_CurrentPosition.getHeading(AngleUnit.RADIANS));
        double sine = Math.sin(m_CurrentPosition.getHeading(AngleUnit.RADIANS));

        double yOutput = (y * cosine) - (x * sine);
        double xOutput = (y * sine) + (x * cosine);

        m_Drive.Drive(yOutput, xOutput, h);

        if (InBounds()){
            m_CurrentState = States.POSITIONED;
        }
    }

    public boolean InPosition(){
        return m_CurrentState == States.POSITIONED;
    }

    private boolean InBounds() {
        boolean xInBounds = m_CurrentPosition.getX(DistanceUnit.MM) > (m_DesiredPosition.getX(DistanceUnit.MM) - m_XYToleranceMM) &&
                m_CurrentPosition.getX(DistanceUnit.MM) < (m_DesiredPosition.getX(DistanceUnit.MM) + m_XYToleranceMM);
        boolean yInBounds = m_CurrentPosition.getY(DistanceUnit.MM) > (m_DesiredPosition.getY(DistanceUnit.MM) - m_XYToleranceMM) &&
                m_CurrentPosition.getY(DistanceUnit.MM) < (m_DesiredPosition.getY(DistanceUnit.MM) + m_XYToleranceMM);
        boolean hInBounds = m_CurrentPosition.getHeading(AngleUnit.RADIANS) > (m_DesiredPosition.getHeading(AngleUnit.RADIANS) - m_HToleranceRadian) &&
                m_CurrentPosition.getHeading(AngleUnit.RADIANS) < (m_DesiredPosition.getHeading(AngleUnit.RADIANS) + m_HToleranceRadian);

        return xInBounds && yInBounds && hInBounds;
    }
}
