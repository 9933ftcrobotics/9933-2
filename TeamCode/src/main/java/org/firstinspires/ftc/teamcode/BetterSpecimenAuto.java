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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "BetterSpecimenAuto", group = "Autonomous")
public class BetterSpecimenAuto extends LinearOpMode {


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

        Pose2d initialPose = new Pose2d(-15, 62, Math.toRadians(-90));
        Pose2d secondPos = new Pose2d(-5, 36, Math.toRadians(-90));
        Pose2d thirdPos = new Pose2d(-5, 45, Math.toRadians(-90));
        Pose2d fourthPos = new Pose2d(-36, 25, Math.toRadians(180));
        Pose2d fifthPos = new Pose2d(-36, 55, Math.toRadians(90));
        Pose2d afterPick = new Pose2d(-45, 25, Math.toRadians(180));
        Pose2d sixthPos = new Pose2d(-58, 50, Math.toRadians(90));
        Pose2d afterScoreOne = new Pose2d(-45, 55, Math.toRadians(90));
        Pose2d seventhPos = new Pose2d(-45, 55, Math.toRadians(90));
        Pose2d finishPos = new Pose2d(-5, 36, Math.toRadians(-90)); //TODO: Will need to change is update Trajectory
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);



        TrajectoryActionBuilder First = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-5, 36));
        //.turnTo(Math.toRadians(45))
        //.strafeTo(new Vector2d(45, 45));
        TrajectoryActionBuilder ScoreFirst = drive.actionBuilder(secondPos)
                .strafeTo(new Vector2d(-5, 45));
        TrajectoryActionBuilder Park = drive.actionBuilder(secondPos)
                .strafeTo(new Vector2d(-55, 55));
        TrajectoryActionBuilder FinishWait = drive.actionBuilder(secondPos)
                .waitSeconds(20);
        /*TrajectoryActionBuilder PickFirst = drive.actionBuilder(thirdPos)
                .splineToLinearHeading(new Pose2d(-36, 25, Math.toRadians(180)), 5);
        TrajectoryActionBuilder DropOff = drive.actionBuilder(fourthPos)
                .lineToYSplineHeading(55, Math.toRadians(90));
        TrajectoryActionBuilder GetSecond = drive.actionBuilder(fifthPos)
                .turnTo(Math.toRadians(180))
                .strafeTo(new Vector2d(-45, 25));
        TrajectoryActionBuilder PickSecond = drive.actionBuilder(afterPick)
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(-58, 50));
        TrajectoryActionBuilder PickLast = drive.actionBuilder(sixthPos)
                .strafeTo(new Vector2d(-45, 45))
                .strafeTo(new Vector2d(-45, 55));
        TrajectoryActionBuilder ScoreLast = drive.actionBuilder(afterScoreOne)
                .strafeTo(new Vector2d(-5, 45))
                .turnTo(Math.toRadians(-90));
        TrajectoryActionBuilder FinishScoreLast = drive.actionBuilder(afterScoreOne)
                .strafeTo(new Vector2d(-5, 36));

        TrajectoryActionBuilder SleepPick = drive.actionBuilder(initialPose)
                .waitSeconds(0.5);
        TrajectoryActionBuilder SleepLong = drive.actionBuilder(initialPose)
                .waitSeconds(2);
        TrajectoryActionBuilder SleepReallyLong = drive.actionBuilder(initialPose)
                .waitSeconds(30);
        TrajectoryActionBuilder VeryShort = drive.actionBuilder(initialPose)
                .waitSeconds(0.5);

        TrajectoryActionBuilder Final = drive.actionBuilder(finishPos)
                .strafeTo(new Vector2d(-5, 45))
                .strafeTo(new Vector2d(-47, 55));*/


        // actions that need to happen on init; for instance, a claw tightening.
        //claw.SetWristRight();

        Action RunFirst = First.build();
        Action scoreFist = ScoreFirst.build();
        Action park = Park.build();
        Action SuperLongWait = FinishWait.build();
        /*Action pickUpFirst = PickFirst.build();
        Action dropFirst = DropOff.build();
        Action getSecond = GetSecond.build();
        Action pickSecond = PickSecond.build();
        Action pickLast = PickLast.build();
        ///Action pickLast = PickLast.build();
        Action scoreLast = ScoreLast.build();
        Action finishLastScore = FinishScoreLast.build();
        Action WaitPick = SleepPick.build();
        Action WaitLong = SleepLong.build();
        Action WaitReallyLong = SleepReallyLong.build();
        Action Finish = Final.build();
        Action Short = VeryShort.build();*/

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                    new InstantAction(claw::SetWristRight),
                        new ParallelAction(
                                arm.upSpecimen(),
                                arm.outSpecimen()
                        ),
                    new ParallelAction(
                            RunFirst,
                            arm.upSpecimen(),
                            arm.outSpecimen()
                    ),
                    new ParallelAction(
                            scoreFist,
                            arm.upSpecimenScore(),
                            arm.outSpecimenScore()
                    ),
                        new ParallelAction(
                                park,
                                arm.upRest(),
                                arm.outRest()
                        ),
                        new ParallelAction(
                                SuperLongWait,
                                arm.upZero(),
                                arm.outZero()
                        )

                )
        );
    }
}