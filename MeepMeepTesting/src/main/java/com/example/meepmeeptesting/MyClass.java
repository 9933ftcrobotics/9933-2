package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-15, 62, Math.toRadians(-90)))
                                .strafeTo(new Vector2d(-55, 55))
                                /*.turnTo(Math.toRadians(45))
                                .strafeTo(new Vector2d(50, 50))
                .strafeTo(new Vector2d(35, 35))
                        .turnTo(Math.toRadians(0))
                .strafeTo(new Vector2d(38, 26))
                /*.strafeTo(new Vector2d(4, 38))
                .strafeTo(new Vector2d(30, 40))
                .turnTo(Math.toRadians(0))
                .strafeTo(new Vector2d(38, 26))
                .turnTo(Math.toRadians(45))
                .strafeTo(new Vector2d(50, 50)) //Ended Here
                .strafeTo(new Vector2d(38, 28))
                .turnTo(Math.toRadians(0))
                .strafeTo(new Vector2d(48, 26))
                .strafeTo(new Vector2d(57, 26))*/
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}