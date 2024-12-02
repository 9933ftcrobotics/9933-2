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
public class MainDrive_CC extends LinearOpMode {
    boolean wristCenter = true;


    // This variable determines whether the following program
    // uses field-centric or robot-centric driving styles. The
    // differences between them can be read here in the docs:
    // https://docs.ftclib.org/ftclib/features/drivebases#control-scheme

    private static ElapsedTime timmer = new ElapsedTime();
    private static ElapsedTime rumble = new ElapsedTime();
    private static ElapsedTime huskyTime = new ElapsedTime();
    private static ElapsedTime reset = new ElapsedTime();
    boolean warning = false;
    boolean endGame = false;

    static final boolean FIELD_CENTRIC = true;

    boolean sampleScoring = true; //true = sample false = specimin

    boolean YIsPressed = false;

    double leftY, leftX, rightX;

    int ReqArmPos, ReqOutPos;

    boolean Arm_Override_Active;

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

        ReqArmPos = DriveConstants.armSampleRest;
        ReqOutPos = DriveConstants.armOutSampleRest;

        waitForStart();
        //camera.initAprilTag();

        while (!isStopRequested()) {
            rightX = driver1.getRightX();
            leftX = driver1.getLeftX();
            leftY = driver1.getLeftY();



            drive.drive(leftX, leftY, rightX, true);

            drive.getCurrentPose();

            updateTelemetry(drive.getDriveTelemetry());
            updateTelemetry(arm.getArmTelemetry());

            CommandScheduler.getInstance().run();

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

            if (!driver1.getButton(GamepadKeys.Button.Y) && !driver2.getButton(GamepadKeys.Button.Y) ) {
                YIsPressed = false;
            }

            claw.SetWristCenter();

            if(!Arm_Override_Active) {
                //by putting this out of the state machine we don't accidentally forget to call this
                // and send the arm into orbit because its still set to a constant power value instead
                // of still checking against the PID controllers.
                arm.setArms(ReqArmPos,ReqOutPos);
                if(driver1.getButton(GamepadKeys.Button.DPAD_UP) || driver1.getButton(GamepadKeys.Button.DPAD_DOWN) || driver1.getButton(GamepadKeys.Button.A) || driver1.getButton(GamepadKeys.Button.Y)){
                    Arm_Override_Active = true;
                }
                else if (driver2.getButton(GamepadKeys.Button.DPAD_UP)) {
                    ReqArmPos = DriveConstants.armSampleScoreHigh;
                    ReqOutPos = DriveConstants.armOutSampleScoreHigh;
                }
                else {
                    ReqArmPos = DriveConstants.armSampleRest;
                    ReqOutPos = DriveConstants.armOutSampleRest;
                }
            }
            else{
                //have to call seperate arms because when overriding you want to be able to lift
                //and lower arm regardless where the out arm is
                arm.setArm(ReqArmPos);
                arm.setOutArm(ReqOutPos);

                if(driver1.getButton(GamepadKeys.Button.BACK)){
                    Arm_Override_Active = false;
                }
                else if(driver1.getButton(GamepadKeys.Button.START)){
                    Arm_Override_Active = false;
                    arm.resetOutArm();
                    arm.resetArm();
                }
                else if(driver1.getButton(GamepadKeys.Button.DPAD_UP)){
                    ReqArmPos = ReqArmPos + 10;
                }
                else if(driver1.getButton(GamepadKeys.Button.DPAD_DOWN)){
                    ReqArmPos = ReqArmPos - 10;
                }
                else if(driver1.getButton(GamepadKeys.Button.Y)){
                    ReqOutPos = ReqOutPos + 20;
                }
                else if(driver1.getButton(GamepadKeys.Button.A)){
                    ReqOutPos = ReqOutPos - 20;
                }
            }
            if (rumble.seconds() > 80 && !warning) {
                driver1.gamepad.rumbleBlips(3);
                warning = true;
            } else if (rumble.seconds() > 90 && !endGame) {
                driver1.gamepad.rumbleBlips(6);
                endGame = true;
            }
            telemetry.addData("Timmer For End Game", rumble);


            telemetry.update();


        }
    }
    public void updateTelemetry(String[] telem) {
        for (String s : telem) {
            telemetry.addLine(s);
        }
    }
}