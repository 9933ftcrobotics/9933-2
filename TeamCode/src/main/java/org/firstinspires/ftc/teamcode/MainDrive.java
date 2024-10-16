package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

@TeleOp
public class MainDrive extends LinearOpMode {

    // This variable determines whether the following program
    // uses field-centric or robot-centric driving styles. The
    // differences between them can be read here in the docs:
    // https://docs.ftclib.org/ftclib/features/drivebases#control-scheme
    static final boolean FIELD_CENTRIC = true;

    boolean sampleScoring = true; //true = sample false = specimin

    boolean YIsPressed = false;

    boolean leftBumperPressed = false;
    boolean rightBumperPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        //CommandScheduler.getInstance().run();

        DriveSubsystem drive = new DriveSubsystem(
                new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "rightRear", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "leftRear", Motor.GoBILDA.RPM_312),
                new RevIMU(hardwareMap)
        );

        ArmSubsystem arm = new ArmSubsystem(
                new Motor(hardwareMap, "arm", Motor.GoBILDA.RPM_312)
        );

        ClawSubsystem claw = new ClawSubsystem(
                new CRServo(hardwareMap, "grabber"),
                new SimpleServo(hardwareMap, "wrist", 0,1)
        );
        // This is the built-in IMU in the REV hub.
        // We're initializing it by its default parameters
        // and name in the config ('imu'). The orientation
        // of the hub is important. Below is a model
        // of the REV Hub and the orientation axes for the IMU.
        //
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // (unapologetically stolen from the road-runner-quickstart)


        // the extended gamepad object
        GamepadEx driverOp = new GamepadEx(gamepad1);

        waitForStart();

        while (!isStopRequested()) {

            drive.drive(driverOp.getLeftX(), driverOp.getLeftY(), driverOp.getRightX(), true);


            /*if (driverOp.getButton(GamepadKeys.Button.B)) {
                arm.setArm(600);
            } else {
                arm.setArm(200);
            }*/

            if (driverOp.getButton(GamepadKeys.Button.Y ) && YIsPressed == false) {
                telemetry.addLine("Y is pressed");
                if (sampleScoring == true) {
                    sampleScoring = false;
                    telemetry.addLine("sampleScoring is true. Changing to false");
                } else if (sampleScoring == false) {
                    sampleScoring = true;
                    telemetry.addLine("sampleScoring is false. Changing to true");
                }
                YIsPressed = true;
            }

            if (!driverOp.getButton(GamepadKeys.Button.Y)) {
                YIsPressed = false;
            }

            if (sampleScoring == true) {
                if (driverOp.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                    arm.setArm(DriveConstants.armSamplePick);
                    claw.grabberPick();
                } else if (driverOp.getButton(GamepadKeys.Button.DPAD_UP)) {
                    arm.setArm(DriveConstants.armSampleScore);
                    claw.grabberStop();
                } else {
                    arm.setArm(DriveConstants.armSampleRest);
                    claw.grabberStop();
                }

                    claw.SetWristCenter();


                if (driverOp.getButton(GamepadKeys.Button.A)) {
                    claw.grabberPlace();
                }

                if (driverOp.getButton(GamepadKeys.Button.B)) {
                    arm.setArm(DriveConstants.armZero);
                }


                if (driverOp.getButton(GamepadKeys.Button.X)) {
                    claw.SetWristCenter();
                }


                telemetry.addLine("Sample Scoring");
            }


            if (sampleScoring == false) {
                if (driverOp.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                    arm.setArm(DriveConstants.armSamplePick);
                    claw.grabberPick();
                } else if (driverOp.getButton(GamepadKeys.Button.DPAD_UP)) {
                    arm.setArm(DriveConstants.armSampleScore);
                    claw.grabberStop();
                }  else if (driverOp.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                    arm.setArm(DriveConstants.armSpecimenOver);
                } else if (driverOp.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                    arm.setArm(DriveConstants.armSpecimenClip);
                } else {
                    arm.setArm(DriveConstants.armSampleRest);
                }


                if (driverOp.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                    claw.SetWristLeft();
                } else if (driverOp.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
                    claw.SetWristRight();
                } else if (driverOp.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                    claw.SetWristCenter();
                }


                if (driverOp.getButton(GamepadKeys.Button.A)) {
                    claw.grabberPlace();
                }

                if (driverOp.getButton(GamepadKeys.Button.B)) {
                    arm.setArm(DriveConstants.armZero);
                }


                /*if (driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER) && !leftBumperPressed) {
                    arm.setArm(DriveConstants.armCurrent -= 50);
                    leftBumperPressed = true;
                } else if (driverOp.getButton(GamepadKeys.Button.RIGHT_BUMPER) && !rightBumperPressed) {
                    arm.setArm(DriveConstants.armCurrent += 50);
                    rightBumperPressed = true;
                }*/

                if (!driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                    leftBumperPressed = false;
                }

                if (!driverOp.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                    rightBumperPressed = false;
                }

                if (driverOp.getButton(GamepadKeys.Button.X)) {
                    claw.SetWristCenter();
                }


                telemetry.addLine("Specimin Scoring");
            }


            telemetry.update();
        }
    }
}