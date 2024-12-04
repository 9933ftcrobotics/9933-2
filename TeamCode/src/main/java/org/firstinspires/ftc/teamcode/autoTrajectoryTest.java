package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "autoTrajectoryTest", group = "Autonomous")

public class autoTrajectoryTest extends LinearOpMode {


    @Override
    public void runOpMode() {
        ArmSubsystem arm = new ArmSubsystem(
                new Motor(hardwareMap, "arm", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "outArm", Motor.GoBILDA.RPM_312)
        );
        ClawSubsystem claw = new ClawSubsystem(
                new CRServo(hardwareMap, "grabber"),
                new SimpleServo(hardwareMap, "wrist", 0,1)
        );

        Pose2d initialPose = new Pose2d(32, 62, Math.toRadians(-90));
        Pose2d finishScorePos = new Pose2d(45, 45, Math.toRadians(50));
        Pose2d afterScorePos = new Pose2d(50, 50, Math.toRadians(50));
        Pose2d pickFirstFinishPos = new Pose2d(35, 37, Math.toRadians(0));
        Pose2d prepSecondScorePos = new Pose2d(35, 27, Math.toRadians(0));
        Pose2d SecondScorePos = new Pose2d(45, 45, Math.toRadians(50));
        Pose2d prepThirdScorePos = new Pose2d(45, 25, Math.toRadians(0));
        Pose2d grabThirdPos = new Pose2d(50, 50, Math.toRadians(50));
        Pose2d ThirdScorePos = new Pose2d(45, 45, Math.toRadians(50));
        Pose2d finishPos = new Pose2d(50, 50, Math.toRadians(50)); //TODO: Will need to change if update Trajectory
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);



        TrajectoryActionBuilder PrepFirstScore = drive.actionBuilder(initialPose)
                .strafeToSplineHeading(new Vector2d(45, 45), Math.toRadians(50));


        TrajectoryActionBuilder FinishScore = drive.actionBuilder(finishScorePos)
                .strafeTo(new Vector2d(50, 50));


        TrajectoryActionBuilder PickFirst = drive.actionBuilder(afterScorePos)
                //.strafeToSplineHeading(new Vector2d(35, 37), Math.toRadians(0))
        //TrajectoryActionBuilder PickFirstFinish = drive.actionBuilder(pickFirstFinishPos)
                //.strafeTo(new Vector2d(35, 25));
                .strafeToSplineHeading(new Vector2d(35, 27), Math.toRadians(0));


        TrajectoryActionBuilder PrepSecondScore = drive.actionBuilder(prepSecondScorePos)
                .strafeToSplineHeading(new Vector2d(45, 45), Math.toRadians(50));

        TrajectoryActionBuilder SecondScore = drive.actionBuilder(SecondScorePos)
                .strafeToSplineHeading(new Vector2d(50, 50), Math.toRadians(50));


        TrajectoryActionBuilder Finish = drive.actionBuilder(finishPos)
                .strafeToSplineHeading(new Vector2d(40, 40), Math.toRadians(-90));


        TrajectoryActionBuilder GrabThird = drive.actionBuilder(grabThirdPos)
            .strafeToSplineHeading(new Vector2d(45, 25), Math.toRadians(0));


        TrajectoryActionBuilder PrepThirdScore = drive.actionBuilder(prepThirdScorePos)
                .strafeToSplineHeading(new Vector2d(45, 45), Math.toRadians(50));

        TrajectoryActionBuilder ThirdScore = drive.actionBuilder(ThirdScorePos)
                .strafeToSplineHeading(new Vector2d(50, 50), Math.toRadians(50));


        TrajectoryActionBuilder WaitScore = drive.actionBuilder(initialPose)
                .waitSeconds(0.25);
        TrajectoryActionBuilder WaitPick = drive.actionBuilder(initialPose)
                .waitSeconds(1);



        // actions that need to happen on init; for instance, a claw tightening.
        //claw.SetWristRight();

        Action prepFirstScoring = PrepFirstScore.build();
        Action finishScore = FinishScore.build();
        Action pickFirst = PickFirst.build();
        //Action pickFirstFinish = PickFirstFinish.build();
        Action prepSecondScore = PrepSecondScore.build();
        Action prepThirdScore = PrepThirdScore.build();
        Action finish = Finish.build();
        Action grabThird = GrabThird.build();
        Action waitPick = WaitPick.build();
        Action waitScore = WaitScore.build();


        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        new InstantAction(claw::SetWristCenter),
                    new ParallelAction( //Prep Score
                            arm.upHigh(),
                            prepFirstScoring,
                            arm.outMid()
                    ),
                    new ParallelAction( //Score
                            arm.upHigh(),
                            finishScore,
                            arm.outHigh()
                    ),
                        new ParallelAction( //Place
                                new InstantAction(claw::grabberPlace),
                                waitScore,
                                arm.outHigh(),
                                arm.upHigh()
                        ),
                        new ParallelAction( //Rest
                                new InstantAction(claw::grabberStop),
                                arm.outRest(),
                                arm.upHigh()
                        ),

                        new ParallelAction( //Drive to second
                                arm.outRest(),
                                arm.upRest(),
                                pickFirst
                        ),
                        new ParallelAction( //Pick second
                                new InstantAction(claw::grabberPick),
                                arm.outPick(),
                                arm.upPick()
                        ),
                        new ParallelAction( //Pick second
                                new InstantAction(claw::grabberPick),
                                arm.outPick(),
                                arm.upPick(),
                                waitPick
                        ),
                        new ParallelAction( //Drive to start score second
                                new InstantAction(claw::grabberStop),
                                arm.outMid(),
                                arm.upHigh(),
                                prepSecondScore
                        ),
                        new ParallelAction( //Drive to finish score second
                                arm.outHigh(),
                                arm.upHigh(),
                                finishScore
                        ),
                        /*new ParallelAction( //Place second
                                arm.outHigh(),
                                arm.upHigh(),
                                new InstantAction(claw::grabberPlace)
                        ),*/
                        new ParallelAction( //Place second
                                arm.outHigh(),
                                arm.upHigh(),
                                new InstantAction(claw::grabberPlace),
                                waitScore
                        ),
                        new ParallelAction( //Rest
                                new InstantAction(claw::grabberStop),
                                arm.outRest(),
                                arm.upHigh()
                        ),

                        new ParallelAction( //Drive to third
                                arm.outRest(),
                                arm.upRest(),
                                grabThird
                        ),
                        new ParallelAction( //Pick third
                                //new InstantAction(claw::grabberPick),
                                arm.outPick(),
                                arm.upPick()
                                //waitPick
                        ),
                        new ParallelAction( //Pick third
                                new InstantAction(claw::grabberPick),
                                arm.outPick(),
                                arm.upPick(),
                                waitPick
                        ),
                        new ParallelAction( //Drive to start score third
                                new InstantAction(claw::grabberStop),
                                arm.outMid(),
                                arm.upHigh(),
                                prepThirdScore
                        ),
                        new ParallelAction( //Drive to finish score third
                                arm.outHigh(),
                                arm.upHigh(),
                                finishScore
                        ),
                        new ParallelAction( //Place third
                                arm.outHigh(),
                                arm.upHigh()
                        ),
                        new ParallelAction( //Place third
                                arm.outHigh(),
                                arm.upHigh(),
                                new InstantAction(claw::grabberPlace),
                                waitScore
                        ),
                        new ParallelAction( //Rest
                                new InstantAction(claw::grabberStop),
                                arm.outRest(),
                                arm.upHigh()
                        ),
                        new ParallelAction(//Finish
                                finish,
                                arm.outZero(),
                                arm.upZero()
                        )

                )
        );
    }
}