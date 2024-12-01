package org.firstinspires.ftc.teamcode;

import static com.acmerobotics.roadrunner.Math.clamp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class armTest extends OpMode {
    private PIDController controller;

    //public static double p = 0.008, i = 0, d = 1;
    //public static double f = 1;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    //public static double p2 = 0.005, i2 = 0, d2 = 0;
    //public static double f2 = 1;

    public static int target = 0;
    public static int outTarget = 0;

    //private final double ticks_in_degree =  5281.1 / 360; //gobilda 30rpm encoder
    private final double ticks_in_degree =  8192 / 360; //rev thru bore

    private DcMotorEx arm_motor,outArm;

    @Override
    public void init(){
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorEx.class, "arm");
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outArm = hardwareMap.get(DcMotorEx.class, "outArm");
        outArm.setDirection(DcMotorSimple.Direction.REVERSE);
        outArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outArm.setTargetPosition(outTarget);
        outArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    @Override
    public void loop()  {

        controller.setPID(p,i,d);
        int armPos = arm_motor.getCurrentPosition();

        double pid = controller.calculate(armPos,target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree))*f;

        double power = pid + ff;
        power = clamp(power,-1,1);
        arm_motor.setPower(power);

        /*controller.setPID(p2,i2,d2);
        int outArmPos = outArm.getCurrentPosition();

        double pidOut = controller.calculate(outArmPos,outTarget);
        double ffOut = Math.cos(Math.toRadians(target / ticks_in_degree))*f;

        double power2 = pidOut + ffOut;

        outArm.setPower(power2);*/

        outArm.setTargetPosition(outTarget);
        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);

        telemetry.addData("power", power);

        telemetry.update();


    }
}

