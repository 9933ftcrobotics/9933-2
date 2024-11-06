package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "autoTest", group = "Autonomous")
public class autoTest extends LinearOpMode {
    public class lift {
        private DcMotorEx lift;

        public lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "ArmUpDown");
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setTargetPosition(0);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public class Rest implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                lift.setTargetPosition(DriveConstants.armSpecimenRest);
                return false;
            }
        }

        public Action rest() {
            return new Rest();
        }
        public class ScoreSpecimen implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                lift.setTargetPosition(DriveConstants.armSpecimenClip);
                sleep(500);
                return false;
            }
        }

        public Action scoreSpecimen() {
            return new ScoreSpecimen();
        }

        public class ScoreSample implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                lift.setTargetPosition(DriveConstants.armSampleScoreHigh);
                sleep(500);
                return false;
            }
        }

        public Action scoreSample() {
            return new ScoreSample();
        }
    }

    public class OutArm {
        private DcMotorEx outarm;

        public OutArm(HardwareMap hardwareMap) {
            outarm = hardwareMap.get(DcMotorEx.class, "ArmInOut");
            outarm.setDirection(DcMotor.Direction.REVERSE);
            outarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            outarm.setTargetPosition(0);
            outarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public class OutRest implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    outarm.setPower(0.8);
                    initialized = true;
                }

                double pos2 = outarm.getCurrentPosition();
                outarm.setTargetPosition(DriveConstants.armOutSampleRest);
                return false;
            }
        }

        public Action outRest() {
            return new OutRest();
        }
        public class OutHigh implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    outarm.setPower(0.8);
                    initialized = true;
                }

                double pos = outarm.getCurrentPosition();
                outarm.setTargetPosition(DriveConstants.armOutSampleScoreHigh);
                return false;
            }
        }

        public Action outHigh() {
            return new OutHigh();
        }

        public class OutSpecimenScore implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    outarm.setPower(0.8);
                    initialized = true;
                }

                double pos = outarm.getCurrentPosition();
                outarm.setTargetPosition(DriveConstants.armOutSpecimenClip);
                return false;
            }
        }

        public Action outSpecimen() {
            return new OutSpecimenScore();
        }
    }



    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(14, 61.25, Math.toRadians(-90)));
        //Lift lift = new Lift(hardwareMap);
        //OutArm Out = new OutArm(hardwareMap);

        // vision here that outputs position
        int visionOutputPosition = 1;

        Action trajectoryAction1;
        Action trajectoryAction2;
        Action trajectoryAction3;
        Action trajectoryAction4;
        Action trajectoryAction5;
        Action trajectoryAction6;
        Action trajectoryAction7;
        Action trajectoryAction8;
        Action trajectoryActionCloseOut;

        trajectoryAction1 = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(new Vector2d(5, 35))
                .build();
        trajectoryAction2 = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(new Vector2d(5, 42))
                .turnTo(Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(32, 42))
                .strafeToConstantHeading(new Vector2d(35, 42))
                .strafeToConstantHeading(new Vector2d(38, 27))
                .build();
        trajectoryAction3 = drive.actionBuilder(drive.pose)
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(40, 36)) // Get first yellow
                .turnTo(Math.toRadians(45))
                .strafeToConstantHeading(new Vector2d(51, 51))
                .build();
        trajectoryAction4 = drive.actionBuilder(drive.pose)
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(35, 42)) //Score
                .turnTo(Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(48, 26))
                .build();
        trajectoryAction5 = drive.actionBuilder(drive.pose)
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(50, 36))
                .turnTo(Math.toRadians(45))
                .strafeToConstantHeading(new Vector2d(51, 51))
                .build();
        trajectoryAction6 = drive.actionBuilder(drive.pose)
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(45, 38))
                .turnTo(Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(58, 24)) // Go to third
                .build();
        trajectoryAction7 = drive.actionBuilder(drive.pose)
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(40, 27))
                .turnTo(Math.toRadians(45))
                .strafeToConstantHeading(new Vector2d(51, 51))
                .build();



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
                        lift.rest(),
                        OutArm.outSpecimen(),
                        trajectoryAction1,
                        lift.liftHigh(),
                        trajectoryAction2,
                        lift.liftLow(),
                        Out.outLow(),
                        trajectoryAction3,
                        lift.liftHigh(),
                        Out.outHigh(),
                        trajectoryAction4,
                        lift.liftLow(),
                        Out.outLow(),
                        trajectoryAction5,
                        lift.liftHigh(),
                        Out.outHigh(),
                        trajectoryAction6,
                        lift.liftLow(),
                        Out.outLow(),
                        trajectoryAction7,
                        lift.liftHigh(),
                        Out.outHigh()

                )
        );
    }
}