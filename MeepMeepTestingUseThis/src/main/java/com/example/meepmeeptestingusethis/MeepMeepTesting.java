package com.example.meepmeeptestingusethis;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .build();
        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-13.72, -61.32, Math.toRadians(90.0)))
                        .splineTo(new Vector2d(10, -38), Math.toRadians(90))
                        .waitSeconds(1.5)
                        .strafeToSplineHeading(new Vector2d(8, -39), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(-51, -58, Math.toRadians(0)), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(0, -52), Math.toRadians(45))
                        .splineToSplineHeading(new Pose2d(44.3, -37.5, Math.toRadians(45)), Math.toRadians(0))
                        .strafeToSplineHeading(new Vector2d(-5, -44), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(-51, -48), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(40, -56), Math.toRadians(45))
                        .splineToSplineHeading(new Pose2d(50, -50, Math.toRadians(30)), Math.toRadians(90))
                        .strafeToSplineHeading(new Vector2d(-5, -44), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(-46, -44), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(-51, -48), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(45, -50), Math.toRadians(0))
                        .build());

        myBot2.runAction(myBot2.getDrive().actionBuilder(new Pose2d(12.09+3.5, -59.84, Math.toRadians(90.0)))
                .splineTo(new Vector2d(10,-38),Math.toRadians(90))
                .waitSeconds(1.5)
                .strafeToSplineHeading(new Vector2d(8,-39),Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-51,-58,Math.toRadians(0)),Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(0,-52),Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(44.3,-37.5,Math.toRadians(45)),Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(-5,-44),Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-51,-48),Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(0,-56),Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(48.3,-37.5,Math.toRadians(45)),Math.toRadians(0))
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                //.addEntity(myBot2)
                .start();
    }
}