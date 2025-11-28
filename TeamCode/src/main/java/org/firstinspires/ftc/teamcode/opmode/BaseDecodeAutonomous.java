package org.firstinspires.ftc.teamcode.opmode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.control.DecodeControl;

public abstract class BaseDecodeAutonomous extends BaseOpMode {
    enum State {
        IDLE,
        GO_TO_LAUNCH_POSITION,
        LAUNCH_ARTIFACTS,
        GO_TO_SPIKE_POSITION,
        INTAKE_SPIKE,
        GO_TO_GATE_POSITION,
        OPEN_GATE,
        FINISHED
    }

    enum Artifact {
        ARTIFACT1,
        ARTIFACT2,
        ARTIFACT3
    }

    enum Spike {
        SPIKE1,
        SPIKE2,
        SPIKE3,
        COMPLETE
    }

    private DecodeControl m_Controls;

    private State m_CurrentState;
    private Artifact m_ArtifactIndex;
    private boolean m_RequestShot;
    private Spike m_SpikeIndex;
    private Spike m_NextSpikeIndex;
    private boolean m_IntakeArtifact;
    private boolean m_OpenGate;


    protected Pose2D m_LaunchPosition;
    protected Pose2D m_Spike1Position;
    protected Pose2D m_Spike2Position;
    protected Pose2D m_Spike3Position;
    protected Pose2D m_OpenGatePosition;

    protected double m_ArtifactLengthIN;
    protected double m_GateForwardDistanceIN;


    @Override
    public void init() {
        super.init();
        m_Controls = new DecodeControl(hardwareMap);
        m_CurrentState = State.IDLE;
        m_ArtifactIndex = Artifact.ARTIFACT1;
        m_SpikeIndex = Spike.SPIKE1;
    }

    @Override
    public void loop() {
        m_Pinpoint.update();
        Pose2D CurrentPose = new Pose2D(DistanceUnit.INCH, -m_Pinpoint.getPosY(DistanceUnit.INCH), m_Pinpoint.getPosX(DistanceUnit.INCH), AngleUnit.DEGREES, -m_Pinpoint.getHeading(AngleUnit.DEGREES));

        m_Localizer.Localize(CurrentPose);

        switch (m_CurrentState) {
            case IDLE:
                m_CurrentState = State.GO_TO_LAUNCH_POSITION;
                break;
            case GO_TO_LAUNCH_POSITION:
                m_Localizer.SetDesiredPosition(m_LaunchPosition);
                m_Controls.launcherSpinUp();
                if (m_Localizer.InPosition()){
                    m_CurrentState = State.LAUNCH_ARTIFACTS;
                    m_ArtifactIndex = Artifact.ARTIFACT1;
                    m_RequestShot = true;
                }
                break;
            case LAUNCH_ARTIFACTS:
                switch (m_ArtifactIndex) {
                    case ARTIFACT1:
                        m_Controls.launchLeft(m_RequestShot);
                        m_RequestShot = false;
                        if (m_Controls.leftLauncherState == DecodeControl.LaunchState.IDLE) {
                            m_ArtifactIndex = Artifact.ARTIFACT2;
                            m_RequestShot = true;
                        }
                        break;
                    case ARTIFACT2:
                        m_Controls.launchRight(m_RequestShot);
                        m_RequestShot = false;
                        if (m_Controls.rightLauncherState == DecodeControl.LaunchState.IDLE) {
                            m_ArtifactIndex = Artifact.ARTIFACT3;
                            m_RequestShot = true;
                        }
                        break;
                    case ARTIFACT3:
                        m_Controls.launchLeft(m_RequestShot);
                        m_RequestShot = false;
                        if (m_Controls.leftLauncherState == DecodeControl.LaunchState.IDLE) {
                            m_Controls.launcherStop();
                            if (m_SpikeIndex == Spike.COMPLETE) {
                                m_CurrentState = State.GO_TO_GATE_POSITION;
                            }
                            else {
                                m_CurrentState = State.GO_TO_SPIKE_POSITION;
                            }
                        }
                        break;
                }
                break;
            case GO_TO_SPIKE_POSITION:
                switch (m_SpikeIndex) {
                    case SPIKE1:
                        m_Localizer.SetDesiredPosition(m_Spike1Position);
                        m_NextSpikeIndex = Spike.SPIKE2;
                        break;
                    case SPIKE2:
                        m_Localizer.SetDesiredPosition(m_Spike2Position);
                        m_NextSpikeIndex = Spike.SPIKE3;
                        break;
                    case SPIKE3:
                        m_Localizer.SetDesiredPosition(m_Spike3Position);
                        m_NextSpikeIndex = Spike.COMPLETE;
                        break;
                }
                if (m_Localizer.InPosition()){
                    m_SpikeIndex = m_NextSpikeIndex;
                    m_CurrentState = State.INTAKE_SPIKE;
                    m_Controls.intakeOn();
                    m_IntakeArtifact = true;
                    m_ArtifactIndex = Artifact.ARTIFACT1;
                    m_Localizer.SetMaxPower(0.25);
                }
                break;
            case INTAKE_SPIKE:
                switch (m_ArtifactIndex) {
                    case ARTIFACT1:
                        if (m_IntakeArtifact) {
                            m_Controls.diverterLeft();
                            m_Localizer.MoveY(m_ArtifactLengthIN, DistanceUnit.INCH);
                            m_IntakeArtifact = false;
                        }
                        if (m_Localizer.InPosition()){
                            m_IntakeArtifact = true;
                            m_ArtifactIndex = Artifact.ARTIFACT2;
                        }
                        break;
                    case ARTIFACT2:
                        if (m_IntakeArtifact) {
                            m_Controls.diverterRight();
                            m_Localizer.MoveY(m_ArtifactLengthIN, DistanceUnit.INCH);
                            m_IntakeArtifact = false;
                        }
                        if (m_Localizer.InPosition()){
                            m_IntakeArtifact = true;
                            m_ArtifactIndex = Artifact.ARTIFACT3;
                        }
                        break;
                    case ARTIFACT3:
                        if (m_IntakeArtifact) {
                            m_Controls.diverterLeft();
                            m_Localizer.MoveY(m_ArtifactLengthIN, DistanceUnit.INCH);
                            m_IntakeArtifact = false;
                        }
                        if (m_Localizer.InPosition()){
                            m_CurrentState = State.GO_TO_LAUNCH_POSITION;
                            m_Localizer.SetMaxPower(1);
                        }
                        break;
                }
                break;
            case GO_TO_GATE_POSITION:
                m_Localizer.SetDesiredPosition(m_OpenGatePosition);
                if (m_Localizer.InPosition()){
                    m_CurrentState = State.OPEN_GATE;
                    m_OpenGate = true;
                }
                break;
            case OPEN_GATE:
                if (m_OpenGate) {
                    m_Localizer.SetMaxPower(0.25);
                    m_Localizer.MoveY(m_GateForwardDistanceIN, DistanceUnit.INCH);
                    m_OpenGate = false;
                }
                if (m_Localizer.InPosition()){
                    m_CurrentState = State.FINISHED;
                }
            case FINISHED:
                m_Localizer.SetIdle();
                break;
        }

        telemetry.addData("State", m_CurrentState);
        telemetry.addData("x_enc", m_Pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("y_enc", m_Pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("h_enc", m_Pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("x_desired", m_Localizer.GetDesiredPosition().getX(DistanceUnit.INCH));
        telemetry.addData("y_desired", m_Localizer.GetDesiredPosition().getY(DistanceUnit.INCH));
        telemetry.addData("h_desired", m_Localizer.GetDesiredPosition().getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }
}