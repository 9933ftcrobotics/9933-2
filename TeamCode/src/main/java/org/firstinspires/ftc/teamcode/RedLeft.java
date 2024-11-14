package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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
@Autonomous(name = "RedLeft", group = "Autonomous")
public class RedLeft extends LinearOpMode {


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

        Pose2d initialPose = new Pose2d(15, 62, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder One = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(4, 38));
        TrajectoryActionBuilder Two = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(30, 40))
                .turnTo(Math.toRadians(0))
                .strafeTo(new Vector2d(38, 26));
        TrajectoryActionBuilder Score = drive.actionBuilder(initialPose)
                .turnTo(Math.toRadians(45))
                .strafeTo(new Vector2d(50, 50));
        TrajectoryActionBuilder Four = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(38, 28))
                .turnTo(Math.toRadians(0))
                .strafeTo(new Vector2d(48, 26));
        TrajectoryActionBuilder Sleep = drive.actionBuilder(initialPose)
                .waitSeconds(0.25);
        TrajectoryActionBuilder SleepLong = drive.actionBuilder(initialPose)
                .waitSeconds(0.5);
        TrajectoryActionBuilder Five = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(48, 26))
                .turnTo(Math.toRadians(0))
                .strafeTo(new Vector2d(57, 26));
        TrajectoryActionBuilder Final = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(40, 40))
                .turnTo(Math.toRadians(-90));


        // actions that need to happen on init; for instance, a claw tightening.


        Action RunFirst = One.build();
        Action RunSecond = Two.build();
        Action RunScore = Score.build();
        Action RunFourth = Four.build();
        Action Wait = Sleep.build();
        Action WaitLong = SleepLong.build();
        Action Finish = Final.build();
        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        //Rotate Wrist
                        claw.SetWristRightAuto(),
                        arm.upArmAuto(DriveConstants.armSpecimenClip),
                        //Wait,
                        arm.upOutAuto(DriveConstants.armOutSpecimenClip),
                        RunFirst,
                        arm.upArmAuto(DriveConstants.armSpecimenClip - 100),
                        //Wait,
                        arm.upOutAuto(DriveConstants.armOutSpecimenClip - 75),
                        //Score
                        claw.grabberPlaceAuto(),
                        RunSecond,
                        //Wrist Center
                        claw.SetWristCenterAuto(),
                        claw.grabberStopAuto(),
                        arm.upArmAuto(DriveConstants.armSamplePick),
                        //Wait,
                        arm.upOutAuto(DriveConstants.armOutSamplePick),
                        //Pick Up
                        claw.grabberPickAuto(),
                        WaitLong,
                        RunScore,
                        arm.upArmAuto(DriveConstants.armSampleScoreHigh),
                        Wait,
                        arm.upOutAuto(DriveConstants.armOutSampleScoreHigh),
                        WaitLong,
                        //Score
                        claw.grabberPlaceAuto(),
                        arm.upOutAuto(DriveConstants.armOutSampleRest),
                        Wait,
                        arm.upArmAuto(DriveConstants.armSampleRest),
                        RunFourth,
                        arm.upArmAuto(DriveConstants.armSamplePick),
                        //Wait,
                        arm.upOutAuto(DriveConstants.armOutSamplePick),
                        WaitLong,
                        RunScore,
                        arm.upArmAuto(DriveConstants.armSampleScoreHigh),
                        Wait,
                        arm.upOutAuto(DriveConstants.armOutSampleScoreHigh),
                        WaitLong,
                        //Score
                        claw.grabberPlaceAuto(),
                        arm.upOutAuto(DriveConstants.armOutSampleRest),
                        Wait,
                        arm.upArmAuto(DriveConstants.armSampleRest),
                        claw.grabberStopAuto(),
                        Finish
                )
        );
    }
}