package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops.Tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
@Disabled
@TeleOp(name = "FieldOrientedDriveTest", group = "Tests")
public class FieldOrientedDriveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), 0));
        waitForStart();
        while (opModeIsActive()) {
            drive.fieldDrive(new Pose2d(new Vector2d(- gamepad1.left_stick_x, - gamepad1.left_stick_y), gamepad1.right_stick_x));
        }
    }
}
