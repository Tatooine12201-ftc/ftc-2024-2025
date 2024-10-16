package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Autonomus;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.tatooine.utils.Alliance.CheckAlliance;

@Autonomous(name = "RedFar")

public class FarFromBasket extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        boolean isRed = CheckAlliance.isRed();
        Action test;
        Pose2d beginPose = new Pose2d(12.09+3.5, -59.84, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        TrajectoryActionBuilder trajectoryRed = drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(7.79, -36.70), Math.toRadians(90))
                .waitSeconds(1)
                .strafeToSplineHeading(new Vector2d(55, -58), Math.toRadians(90))
                .waitSeconds(1)
                .turnTo(Math.toRadians(70))
                .waitSeconds(1)
                .turnTo(Math.toRadians(90))
                .waitSeconds(1)
                .turnTo(Math.toRadians(105))
                .waitSeconds(1);
        TrajectoryActionBuilder trajectoryBlue = drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(7.79, -36.70), Math.toRadians(90))
                .waitSeconds(1)
                .strafeToSplineHeading(new Vector2d(55, -58), Math.toRadians(90))
                .waitSeconds(1)
                .turnTo(Math.toRadians(70))
                .waitSeconds(1)
                .turnTo(Math.toRadians(90))
                .waitSeconds(1)
                .turnTo(Math.toRadians(105))
                .waitSeconds(1);

        if (isRed) {
            test = trajectoryRed.build();
        }
        else {
            test = trajectoryBlue.build();
        }

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(1),
                        test,
                        new SleepAction(1)
                )
        );


    }
}