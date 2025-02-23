package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Camera;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Wrist;

import java.util.ArrayList;
import java.util.List;
@Disabled
@TeleOp(name = "WristCameraTest",group = "Tests")
public class WristCameraTest extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
      Wrist wrist = new Wrist(this, true);
      Camera camera = new Camera(this, true, false);
        waitForStart();
        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            if (gamepad1.cross) {
                wrist.setPosAng(1);
            } else if (gamepad1.circle) {
                wrist.setPosAng(0);
            }
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {

                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            dash.sendTelemetryPacket(packet);
            telemetry.update();

        }
    }
}
