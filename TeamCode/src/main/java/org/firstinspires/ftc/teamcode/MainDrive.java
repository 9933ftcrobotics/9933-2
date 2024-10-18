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

    boolean sampleScoring; //true = sample false = specimin

    boolean YIsPressed = false;

    boolean leftBumperPressed = false;
    boolean rightBumperPressed = false;

    boolean started = false;

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
        GamepadEx driver1 = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);


        waitForStart();

        while (!isStopRequested()) {

            drive.drive(driver1.getLeftX(), driver1.getLeftY(), driver1.getRightX(), true);


            /*if (driver1.getButton(GamepadKeys.Button.B)) {
                arm.setArm(600);
            } else {
                arm.setArm(200);
            }*/
            if (!started) {
                arm.setArm(500);
                claw.SetWristCenter();
            }

            if (driver1.getButton(GamepadKeys.Button.A) && !started || driver2.getButton(GamepadKeys.Button.A) && !started) {
                arm.setArm(DriveConstants.armSampleRest);
                sampleScoring = true;
                started = true;
            }

            if (started == true) {

                if (driver1.getButton(GamepadKeys.Button.Y) && YIsPressed == false || driver2.getButton(GamepadKeys.Button.Y) && YIsPressed == false) {
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

                if (!driver1.getButton(GamepadKeys.Button.Y) && !driver2.getButton(GamepadKeys.Button.Y)) {
                    YIsPressed = false;
                }

                if (sampleScoring == true) {
                    if (driver1.getButton(GamepadKeys.Button.DPAD_DOWN) || driver2.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                        arm.setArm(DriveConstants.armSamplePick);
                        claw.grabberPick();
                    } else if (driver1.getButton(GamepadKeys.Button.DPAD_UP) || driver2.getButton(GamepadKeys.Button.DPAD_UP)) {
                        arm.setArm(DriveConstants.armSampleScore);
                        claw.grabberStop();
                    } else if (driver1.getButton(GamepadKeys.Button.B) || driver2.getButton(GamepadKeys.Button.B)) {
                        arm.setArm(DriveConstants.armZero);
                    } else if (driver1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                        arm.setArm(DriveConstants.armStartHang);
                    } else if (driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                        arm.setArm(DriveConstants.armFinishHang);
                    } else{
                        arm.setArm(DriveConstants.armSampleRest);
                        claw.grabberStop();
                    }

                    claw.SetWristCenter();


                    if (driver1.getButton(GamepadKeys.Button.A) || driver2.getButton(GamepadKeys.Button.A)) {
                        claw.grabberPlace();
                    }


                    telemetry.addLine("Sample Scoring");
                }


                if (sampleScoring == false) {
                    if (driver1.getButton(GamepadKeys.Button.DPAD_DOWN) || driver2.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                        arm.setArm(DriveConstants.armSamplePick);
                        claw.grabberPick();
                    } else if (driver1.getButton(GamepadKeys.Button.DPAD_UP) || driver2.getButton(GamepadKeys.Button.DPAD_UP)) {
                        arm.setArm(DriveConstants.armSampleScore);
                        claw.grabberStop();
                    } else if (driver1.getButton(GamepadKeys.Button.B) || driver2.getButton(GamepadKeys.Button.B)) {
                        arm.setArm(DriveConstants.armZero);
                        claw.grabberStop();
                    } else if (driver1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON) || driver2.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                        arm.setArm(DriveConstants.armSpecimenClip);
                        claw.grabberStop();
                    } else if (!driver1.getButton(GamepadKeys.Button.A) && !driver2.getButton(GamepadKeys.Button.A)) {
                        arm.setArm(DriveConstants.armSpecimenRest);
                        claw.grabberStop();
                    }


                    if (driver1.getButton(GamepadKeys.Button.DPAD_LEFT) || driver2.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                        claw.SetWristLeft();
                    } else if (driver1.getButton(GamepadKeys.Button.DPAD_RIGHT) || driver2.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
                        claw.SetWristRight();
                    } else if (driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER) || driver2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                        claw.SetWristCenter();
                    }


                    if (driver1.getButton(GamepadKeys.Button.A) || driver2.getButton(GamepadKeys.Button.A)) {
                        claw.grabberPlace();
                    }



                    if (!driver1.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                        leftBumperPressed = false;
                    }

                    if (!driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                        rightBumperPressed = false;
                    }

                    if (driver1.getButton(GamepadKeys.Button.X)) {
                        claw.SetWristCenter();
                    }


                    telemetry.addLine("Specimen Scoring");
                }
            }


            telemetry.update();
        }
    }
}