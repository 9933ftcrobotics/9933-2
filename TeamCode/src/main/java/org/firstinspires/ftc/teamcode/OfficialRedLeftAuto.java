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
@Autonomous(name = "Official RED LEFT Auto", group = "Autonomous")

public class OfficialRedLeftAuto extends LinearOpMode {


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
        Pose2d afterScorePos = new Pose2d(52, 51, Math.toRadians(50));
        Pose2d afterFirstScorePos = new Pose2d(34, 22, Math.toRadians(0));
        Pose2d pickFirstFinishPos = new Pose2d(35, 37, Math.toRadians(0));
        Pose2d prepSecondScorePos = new Pose2d(38, 22, Math.toRadians(0));
        Pose2d SecondScorePos = new Pose2d(33.5, 22, Math.toRadians(0));
        Pose2d prepThirdScorePos = new Pose2d(50, 22, Math.toRadians(0));
        Pose2d grabThirdPos = new Pose2d(53, 53, Math.toRadians(50));
        Pose2d grabThirdFinishPos = new Pose2d(45.5, 23, Math.toRadians(0));
        Pose2d ThirdScorePos = new Pose2d(44, 22, Math.toRadians(0));
        Pose2d FourthScorePos = new Pose2d(54.5, 22, Math.toRadians(0));
        Pose2d pickFourthPos = new Pose2d(53, 53, Math.toRadians(50));
        Pose2d pickFourthFinishPos = new Pose2d(55.5, 23, Math.toRadians(0));
        Pose2d finishPos = new Pose2d(53, 53, Math.toRadians(50)); //TODO: Will need to change if update Trajectory
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);



        TrajectoryActionBuilder PrepFirstScore = drive.actionBuilder(initialPose)
                .strafeToSplineHeading(new Vector2d(45, 45), Math.toRadians(50));


        TrajectoryActionBuilder FinishScoreOne = drive.actionBuilder(finishScorePos)
                .strafeToSplineHeading(new Vector2d(53, 51), Math.toRadians(50));


        TrajectoryActionBuilder PickFirst = drive.actionBuilder(afterScorePos)
                //.strafeToSplineHeading(new Vector2d(35, 37), Math.toRadians(0))
                //TrajectoryActionBuilder PickFirstFinish = drive.actionBuilder(pickFirstFinishPos)
                //.strafeTo(new Vector2d(35, 25));
                .strafeToSplineHeading(new Vector2d(33.5, 22), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(35.5, 22), Math.toRadians(0));
        TrajectoryActionBuilder PickFirstFinish = drive.actionBuilder(afterFirstScorePos)
                //.strafeToSplineHeading(new Vector2d(35, 37), Math.toRadians(0))
                //TrajectoryActionBuilder PickFirstFinish = drive.actionBuilder(pickFirstFinishPos)
                //.strafeTo(new Vector2d(35, 25));
                .strafeToSplineHeading(new Vector2d(40, 22), Math.toRadians(0));


        TrajectoryActionBuilder PrepSecondScore = drive.actionBuilder(prepSecondScorePos)
                .strafeToSplineHeading(new Vector2d(45, 45), Math.toRadians(50));

        TrajectoryActionBuilder SecondScore = drive.actionBuilder(SecondScorePos)
                .strafeToSplineHeading(new Vector2d(54, 52.9), Math.toRadians(50));


        TrajectoryActionBuilder Finish = drive.actionBuilder(finishPos)
                .strafeToSplineHeading(new Vector2d(40, 40), Math.toRadians(-90));


        TrajectoryActionBuilder GrabThird = drive.actionBuilder(grabThirdPos)
                .strafeToSplineHeading(new Vector2d(42, 22), Math.toRadians(0));
        TrajectoryActionBuilder PickThirdFinish = drive.actionBuilder(grabThirdFinishPos)
                //.strafeToSplineHeading(new Vector2d(35, 37), Math.toRadians(0))
                //TrajectoryActionBuilder PickFirstFinish = drive.actionBuilder(pickFirstFinishPos)
                //.strafeTo(new Vector2d(35, 25));
                .strafeToSplineHeading(new Vector2d(50, 22), Math.toRadians(0));


        TrajectoryActionBuilder PrepThirdScore = drive.actionBuilder(prepThirdScorePos)
                .strafeToSplineHeading(new Vector2d(45, 45), Math.toRadians(50));

        TrajectoryActionBuilder ThirdScore = drive.actionBuilder(ThirdScorePos)
                .strafeToSplineHeading(new Vector2d(52.2, 52.2), Math.toRadians(50));

        TrajectoryActionBuilder PickFourth = drive.actionBuilder(pickFourthPos)
                .strafeToSplineHeading(new Vector2d(52, 24), Math.toRadians(0));
        TrajectoryActionBuilder PickFourthFinish = drive.actionBuilder(pickFourthFinishPos)
                .strafeToSplineHeading(new Vector2d(58, 24), Math.toRadians(0));

        TrajectoryActionBuilder FourthScore = drive.actionBuilder(FourthScorePos)
                .strafeToSplineHeading(new Vector2d(53.5, 52.75), Math.toRadians(50));

        TrajectoryActionBuilder WaitScore = drive.actionBuilder(initialPose)
                .waitSeconds(0.4);
        TrajectoryActionBuilder MidWait = drive.actionBuilder(initialPose)
                .waitSeconds(1);
        TrajectoryActionBuilder WaitPick = drive.actionBuilder(initialPose)
                .waitSeconds(0.7);
        TrajectoryActionBuilder WaitLong = drive.actionBuilder(initialPose)
                .waitSeconds(30);



        // actions that need to happen on init; for instance, a claw tightening.
        //claw.SetWristRight();

        Action prepFirstScoring = PrepFirstScore.build();
        Action finishScore = FinishScoreOne.build();
        Action pickFirst = PickFirst.build();
        Action pickFirstFinish = PickFirstFinish.build();
        //Action pickFirstFinish = PickFirstFinish.build();
        Action prepSecondScore = PrepSecondScore.build();
        Action prepThirdScore = PrepThirdScore.build();
        Action finish = Finish.build();
        Action grabThird = GrabThird.build();
        Action grabThirdFinish = PickThirdFinish.build();
        Action thirdScore = ThirdScore.build();
        Action secondScore = SecondScore.build();
        Action waitPick = WaitPick.build();
        Action waitScore = WaitScore.build();
        Action waitLong = WaitLong.build();
        Action waitMid = MidWait.build();

        Action fourthPick = PickFourth.build();
        Action fourthPickFinish = PickFourthFinish.build();
        Action fourthScore = FourthScore.build();


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
                                arm.outHigh(),
                                finishScore
                        ),
                        /*new ParallelAction( //Place First
                                arm.outHigh(),
                                arm.upHigh()
                        ),*/
                        //waitScore,
                        new ParallelAction( //Place First
                                arm.outHigh(),
                                arm.upHigh(),
                                new InstantAction(claw::grabberPlaceSlow),
                                waitScore
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
                                arm.outPickFar(),
                                arm.upPickFar()
                        ),
                        /*new ParallelAction( //Pick second
                                new InstantAction(claw::grabberPick),
                                arm.outPick(),
                                arm.upPick(),
                                pickFirstFinish
                        ),*/
                        new ParallelAction( //Pick second
                                new InstantAction(claw::grabberPick),
                                arm.outPickFar(),
                                arm.upPickFar(),
                                waitPick
                        ),
                        /*new ParallelAction( //Pick second
                                new InstantAction(claw::grabberPick),
                                arm.outPickFar(),
                                arm.upPickFar(),
                                waitPick
                        ),*/
                        new ParallelAction( //Drive to finish score second
                                //arm.outMid(),
                                arm.upHigh(),
                                secondScore
                        ),
                        new InstantAction(claw::grabberStop),
                        new ParallelAction( //Place second
                                arm.outHigh(),
                                arm.upHigh()
                                //new InstantAction(claw::grabberPlace),
                                //waitScore
                        ),
                        waitScore,
                        new ParallelAction( //Place second
                                arm.outHigh(),
                                arm.upHigh(),
                                new InstantAction(claw::grabberPlace),
                                waitScore
                        ),
                        new ParallelAction( //Rest
                                //new InstantAction(claw::grabberStop),
                                arm.outRest(),
                                arm.upHigh()
                        ),
                        waitScore,
                        new InstantAction(claw::grabberPick),
                        new ParallelAction( //Drive to third
                                arm.outRest(),
                                arm.upRest(),
                                grabThird
                        ),
                        new ParallelAction( //Pick third
                                //new InstantAction(claw::grabberPick),
                                arm.outPick(),
                                arm.upPick(),
                                grabThirdFinish
                                //waitPick
                        ),
                        waitPick,
                        new ParallelAction( //Pick third
                                arm.outPick(),
                                arm.upPick(),
                                waitPick
                        ),
                        new ParallelAction( //Drive to finish score third
                                new InstantAction(claw::grabberStop),
                                //arm.outMid(),
                                arm.upHigh(),
                                thirdScore

                        ),
                        new ParallelAction( //Place third
                                arm.outHigh(),
                                arm.upHigh()
                        ),
                        waitScore,
                        new ParallelAction( //Place third
                                arm.outHigh(),
                                arm.upHigh(),
                                new InstantAction(claw::grabberPlace),
                                waitScore
                        ),
                        new ParallelAction( //Rest
                                new InstantAction(claw::grabberStop),
                                arm.outPick(),
                                arm.upHigh()
                        ),
                        new InstantAction(claw::grabberPick),
                        waitPick,
                        new ParallelAction( //drive to fourth
                                arm.outRest(),
                                arm.upRest(),
                                fourthPick
                        ),
                        new ParallelAction( //Pick fourth
                                new InstantAction(claw::grabberPick),
                                arm.outPick(),
                                arm.upPick(),
                                fourthPickFinish
                        ),
                        new InstantAction(claw::grabberPick),
                        waitMid,
                        new ParallelAction( //Drive to place fourth
                                //arm.outMid(),
                                arm.upHigh(),
                                fourthScore
                        ),

                        new ParallelAction( //Place fourth
                                new InstantAction(claw::grabberStop),
                                arm.outHigh(),
                                arm.upHigh()
                        ),
                        waitScore,
                        new ParallelAction( //Place fourth
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
                        ),
                        new ParallelAction(//Finish
                                arm.outZero(),
                                arm.upZero(),
                                waitLong
                        )

                )
        );
    }
}