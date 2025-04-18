package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Arm;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Intake;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Wrist;
import org.firstinspires.ftc.teamcode.tatooine.utils.States.Conts;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.EasyGamepad;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.GamepadKeys;
import org.firstinspires.ftc.teamcode.tatooine.utils.mathUtil.MathUtil;

import java.util.ArrayList;
import java.util.List;
@Disabled
@TeleOp(name = "ATO", group = "TeleOps")
public class ActionTeleOp extends LinearOpMode {
    private List<Action> runningActions = new ArrayList<>();

    private boolean dPadRightToggle = false;

    private boolean dPadLeftToggle = false;

    private boolean dPadDownToggle = false;

    private double extend = 0;

    private double angle = -5;

    private double lastExtend = 0;

    private final  double EXTEND_TOLERANCE= 2;

    private boolean intaking = false;

    private boolean specimen = false;

    private boolean circleToggle = false;

    private boolean triangleToggle = false;

    private boolean dPadUpToggle = false;

    private Conts conts = new Conts();
    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Arm arm = new Arm(this, true);
        Wrist wrist = new Wrist(this, false);
        Intake intake = new Intake(this, false);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0, Math.toRadians(0)));
        EasyGamepad gamepadEx1 = new EasyGamepad(gamepad1);
        EasyGamepad gamepadEx2 = new EasyGamepad(gamepad2);
        Pose2d beginPose = new Pose2d(6+(18.0/2.0) - 30, -(72-(18.0/2.0)), Math.toRadians(90));
        TrajectoryActionBuilder goScoreSampleFromPark = drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(-63,-61, Math.toRadians(45)), Math.toRadians(-90));
        TrajectoryActionBuilder goScoreSampleFromHang = drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(-63,-61, Math.toRadians(45)), Math.toRadians(-90));
        Action chosenPath = goScoreSampleFromPark.build();
        waitForStart();
        while (opModeIsActive()){
            TelemetryPacket packet = new TelemetryPacket();

            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;


            if (gamepadEx1.getButton(GamepadKeys.Button.START)){
                drive.resetIMU();
            }
            if (intaking) {
                drive.fieldDrive(new Pose2d(-MathUtil.applyDeadzone(gamepadEx1.getStick(GamepadKeys.Stick.LEFT_STICK_X), 0.1), MathUtil.applyDeadzone(gamepadEx1.getStick(GamepadKeys.Stick.LEFT_STICK_Y), 0.1), MathUtil.applyDeadzone(gamepadEx1.getStick(GamepadKeys.Stick.RIGHT_STICK_X), 0.1)));
            }
            if (gamepadEx1.justPressedButton(GamepadKeys.Button.DPAD_UP)){
                runningActions.add(chosenPath);
            }
            //score sample button
            if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_UP) && !dPadUpToggle){
                intaking = false;
                dPadUpToggle = true;
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.moveExtend());
                runningActions.add(new SequentialAction(
                        new ParallelAction(arm.setAngle(Conts.angleScoreSample), arm.setExtend(Conts.extendScoreSampleHigh))
                        , new InstantAction(wrist::doc),new SleepAction(0.2),new InstantAction(()-> intake.setPower(-1))));
                }
            else if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_UP) && dPadUpToggle){
                dPadUpToggle = false;
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.moveExtend());
                runningActions.add(new SequentialAction(intake.stop(),new InstantAction(()-> wrist.intakeFlat()), arm.setExtend(0), arm.setAngle(Conts.angleDrive)));
                intaking = false;
            } else if (gamepadEx2.justPressedButton(GamepadKeys.Button.START)) {
                intaking = !intaking;
                if (intaking){
                    runningActions.clear();
                    runningActions.add(arm.moveAngle());
                    runningActions.add(arm.moveExtend());
                    runningActions.add(intake.intake());
                }
            }
            if (intaking){
                arm.setPowerAngleWithF(MathUtil.applyDeadzone(gamepadEx2.getStick(GamepadKeys.Stick.LEFT_STICK_Y),0.1));
                arm.setPowerExtend(MathUtil.applyDeadzone(gamepadEx2.getStick(GamepadKeys.Stick.RIGHT_STICK_Y), 0.1));
                if (gamepadEx2.justPressedButton(GamepadKeys.Button.CIRCLE)) {
                    wrist.intakeUp();
                } else if (gamepadEx2.justPressedButton(GamepadKeys.Button.TRIANGLE)) {
                    wrist.intakeFlat();
                } else if (gamepadEx2.justPressedButton(GamepadKeys.Button.SQUARE)) {
                    wrist.doc();
                }
            }


            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            gamepadEx1.update(gamepad1);
            gamepadEx2.update(gamepad2);
            telemetry.update();
        }

    }
}
