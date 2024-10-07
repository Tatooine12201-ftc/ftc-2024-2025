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

@Autonomous(name = "BlueFar")
public class BlueFar extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-13.72, -61.32, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        TrajectoryActionBuilder trajectoryAction1 = drive.actionBuilder(beginPose)
                .splineToConstantHeading(new Vector2d(-9.71, -34.78), Math.toRadians(81.42))
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(-32, -39))
                .splineToSplineHeading(new Pose2d(-58, -53, Math.toRadians(-135)), Math.toRadians(270))
                .waitSeconds(1)
                .turnTo(Math.toRadians(70))
                .waitSeconds(1)
                .turnTo(Math.toRadians(-135))
                .waitSeconds(1)
                .turnTo(Math.toRadians(90))
                .waitSeconds(1)
                .turnTo(Math.toRadians(-135))
                .waitSeconds(1)
                .turnTo(Math.toRadians(110))
                .waitSeconds(1)
                .turnTo(Math.toRadians(-135))
                .strafeToSplineHeading(new Vector2d(-30, 0.07), Math.toRadians(0));


        //.splineTo(new Vector2d(-36.70, -63.40), Math.toRadians(202.26));


        Action test = trajectoryAction1.build();


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