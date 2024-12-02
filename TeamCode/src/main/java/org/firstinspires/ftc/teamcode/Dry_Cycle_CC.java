package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem_CC;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@TeleOp
public class Dry_Cycle_CC extends LinearOpMode {
    boolean wristCenter = true;


    // This variable determines whether the following program
    // uses field-centric or robot-centric driving styles. The
    // differences between them can be read here in the docs:
    // https://docs.ftclib.org/ftclib/features/drivebases#control-scheme

    private static ElapsedTime timer = new ElapsedTime();
    private static ElapsedTime rumble = new ElapsedTime();
    private static ElapsedTime huskyTime = new ElapsedTime();
    private static ElapsedTime reset = new ElapsedTime();
    double prev_time, cycles;
    int state;
    boolean warning = false;
    boolean endGame = false;

    static final boolean FIELD_CENTRIC = true;

    boolean sampleScoring = true; //true = sample false = specimin

    boolean YIsPressed = false;

    double leftY, leftX, rightX;

    @Override
    public void runOpMode() throws InterruptedException {

        // constructor takes in frontLeft, frontRight, backLeft, backRight motors
        // IN THAT ORDER

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DriveSubsystem drive = new DriveSubsystem(
                new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "rightRear", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "leftRear", Motor.GoBILDA.RPM_312),
                new RevIMU(hardwareMap),
                hardwareMap.get(HuskyLens.class, "huskyLens")
        );

        ArmSubsystem_CC arm = new ArmSubsystem_CC(
                new Motor(hardwareMap, "arm", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "outArm", Motor.GoBILDA.RPM_312)
        );

        ClawSubsystem claw = new ClawSubsystem(
                new CRServo(hardwareMap, "grabber"),
                new SimpleServo(hardwareMap, "wrist", 0,1)
        );
        CameraSubsystem camera = new CameraSubsystem(

        );

        drive.setReadType(); //Set Husky Cam to color mode
        arm.resetOutArm();
        rumble.reset();


        // the extended gamepad object
        GamepadEx driver1 = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);


        waitForStart();
        //camera.initAprilTag();

        while (!isStopRequested()) {

            CommandScheduler.getInstance().run();


            claw.SetWristCenter();
            //drive.drive(leftX, leftY, rightX, true);

            switch(state){
                case 0:
                    prev_time = timer.seconds();
                    state = 10;
                    break;
                case 10:
                    arm.setArms(DriveConstants.armSampleRest,DriveConstants.armOutSampleRest);
                    if((prev_time+5) < timer.seconds()){
                        state = 20;
                    }
                    break;
                case 20:
                    prev_time = timer.seconds();
                    state = 30;
                    break;
                case 30:
                    arm.setArms(DriveConstants.armSampleScoreHigh,DriveConstants.armOutSampleScoreHigh);
                    if((prev_time+5) < timer.seconds()){
                        state = 0;
                        cycles = cycles + 1;
                    }
                    break;
            }

            if (rumble.seconds() > 80 && !warning) {
                driver1.gamepad.rumbleBlips(3);
                warning = true;
            } else if (rumble.seconds() > 90 && !endGame) {
                driver1.gamepad.rumbleBlips(6);
                endGame = true;
            }
            telemetry.addData("Timmer For End Game", rumble);


            drive.getCurrentPose();
            telemetry.addData("Cycles: ", cycles);
            //updateTelemetry(drive.getDriveTelemetry());
            updateTelemetry(arm.getArmTelemetry());
            telemetry.update();


        }
    }
    public void updateTelemetry(String[] telem) {
        for (String s : telem) {
            telemetry.addLine(s);
        }
    }
}