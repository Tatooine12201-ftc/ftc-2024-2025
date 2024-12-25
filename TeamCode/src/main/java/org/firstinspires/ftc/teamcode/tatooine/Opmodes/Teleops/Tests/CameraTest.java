package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops.Tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Camera;

@TeleOp(name = "CameraTest",group = "Tests")
public class CameraTest extends LinearOpMode {


    @Override
    public void runOpMode() {
        Camera camera = new Camera(this, true, true);
       // Servo s = hardwareMap.get(Servo.class, "s");
        waitForStart();
        while (opModeIsActive()) {
            //s.setPosition(camera.getAngle());
            camera.getAngle();
            if (gamepad1.cross){
                camera.setSpecimen(true);
            } else if (gamepad1.circle) {
                camera.setSpecimen(false);
            }
            telemetry.update();

        }
    }
}
