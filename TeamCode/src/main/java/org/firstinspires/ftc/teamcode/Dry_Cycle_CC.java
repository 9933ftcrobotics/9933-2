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

    private static ElapsedTime timer = new ElapsedTime();

    double prev_time, cycles, TimeElapsed;
    int state, ReqArmPos, ReqOutPos;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        ArmSubsystem_CC arm = new ArmSubsystem_CC(
                new Motor(hardwareMap, "arm", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "outArm", Motor.GoBILDA.RPM_312)
        );

        ClawSubsystem claw = new ClawSubsystem(
                new CRServo(hardwareMap, "grabber"),
                new SimpleServo(hardwareMap, "wrist", 0,1)
        );

        arm.resetOutArm();

        ReqArmPos = DriveConstants.armSampleRest;
        ReqOutPos = DriveConstants.armOutSampleRest;

        waitForStart();

        while (!isStopRequested()) {

            CommandScheduler.getInstance().run();

            //by putting this out of the state machine we don't accidentally forget to call this
            // and send the arm into orbit because its still set to a constant power value instead
            // of still checking against the PID controllers.
            arm.setArms(ReqArmPos,ReqOutPos);


            TimeElapsed = timer.seconds() - prev_time;

            switch(state){

                //Starting State to bring wrist to center in case it starts in tucked starting position
                case 0:
                    claw.SetWristCenter();
                    prev_time = timer.seconds();
                    state = 10;
                    break;

                //Arm to rest position, wait 3s, turn wrist right
                case 10:
                    ReqArmPos = DriveConstants.armSampleRest;
                    ReqOutPos =DriveConstants.armOutSampleRest;
                    if(TimeElapsed > 3){
                        claw.SetWristRight();
                        prev_time = timer.seconds();
                        state = 20;
                    }
                    break;

                //wait 2s, return wrist to center
                case 20:
                    if(TimeElapsed > 2){
                        claw.SetWristCenter();
                        prev_time = timer.seconds();
                        state = 30;
                    }
                    break;

                //wait 2s
                case 30:
                    if(TimeElapsed > 2){
                        prev_time = timer.seconds();
                        state = 40;
                    }
                    break;

                //Raise arm, wait 5s, set wrist right
                case 40:

                    ReqArmPos = DriveConstants.armSampleScoreHigh;
                    ReqOutPos =DriveConstants.armOutSampleScoreHigh;

                    if(TimeElapsed > 5){
                        prev_time = timer.seconds();
                        claw.SetWristRight();
                        state = 50;
                    }
                    break;

                //wait 2s, wrist back to center
                case 50:
                    if(TimeElapsed > 2){
                        prev_time = timer.seconds();
                        claw.SetWristCenter();
                        state = 60;
                    }
                    break;

                //wait 2s, count cycle and restart back
                case 60:
                    if(TimeElapsed > 2){
                        prev_time = timer.seconds();
                        cycles = cycles + 1;
                        state = 10;
                    }
                    break;
            }


            telemetry.addData("Timer", timer);
            telemetry.addData("Prev Time", prev_time);
            telemetry.addData("Elapsed Time", TimeElapsed);

            telemetry.addData("Cycles: ", cycles);

            telemetry.addData("State: ", state);
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