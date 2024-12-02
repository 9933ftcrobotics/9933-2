package org.firstinspires.ftc.teamcode.subsystems;

import static com.acmerobotics.roadrunner.Math.clamp;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.DriveConstants;

public class ArmSubsystem_CC extends SubsystemBase {

    private Motor arm;
    private Motor outArm;

    double outPower = 0;

    private PIDController armPID,outPID;

    public static int outCurrent;
    public static int armCurrent;

    public static int ArmTarget = 0;
    public static int OutTarget = 0;

    private final double ticks_in_degree =  5281.1 / 360; //gobilda 30rpm encoder
    //private final double ticks_in_degree =  8192 / 360; //rev thru bore

    public static double arm_p = 0.0125, arm_i = 0.06, arm_d = 0.00075;
    public static double arm_f = 0.08;

    public static double out_p = 0.004, out_i = 0, out_d = 0;

    public ArmSubsystem_CC(Motor arm, Motor outArm) {
        arm.setInverted(false);
        outArm.setInverted(true);
        this.arm = arm;
        this.outArm = outArm;
        arm.resetEncoder();
        outArm.resetEncoder();
        armPID = new PIDController(arm_p,arm_i,arm_d);
        outPID = new PIDController(out_p,out_i,out_d);

    }

    public void setArms(int ReqArmTarget, int ReqOutTarget) {
        armCurrent = arm.getCurrentPosition();
        outCurrent = outArm.getCurrentPosition();

        if(outCurrent < 300) {
            ArmTarget = ReqArmTarget;
        }

        if(Math.abs(armCurrent-ArmTarget)<300){
            OutTarget = ReqOutTarget;
        }

        armPID.setPID(arm_p,arm_i,arm_d);

        double armpid = armPID.calculate(armCurrent,ArmTarget);
        double armff = Math.cos(Math.toRadians(ArmTarget / ticks_in_degree))*arm_f;

        double power = armpid + armff;

        power = clamp(power,-0.4,1);
        arm.set(power);

        outPID.setPID(out_p,out_i,out_d);

        double outpid = outPID.calculate(outCurrent,OutTarget);
        outpid = clamp(outpid,-0.65,0.65);
        outArm.set(outpid);


    }
    public void setArm(int ReqArmPos) {

        armPID.setPID(arm_p, arm_i, arm_d);
        armCurrent = arm.getCurrentPosition();
        ArmTarget = ReqArmPos;
        double armpid = armPID.calculate(armCurrent, ArmTarget);
        double armff = Math.cos(Math.toRadians(ArmTarget / ticks_in_degree)) * arm_f;


        double power = armpid + armff;

        power = clamp(power,-0.4,1);
        arm.set(power);
    }

    public void setOutArm(int outPos) {
        outPID.setPID(out_p,out_i,out_d);
        outCurrent = outArm.getCurrentPosition();
        OutTarget = outPos;

        double outpid = outPID.calculate(outCurrent,OutTarget);
        outpid = clamp(outpid,-0.65,0.65);
        outArm.set(outpid);

    }

    public void powerArm(double Power) {
        arm.set(Power);
    }

    public void powerOutArm(double Power) {
        outArm.set(Power);
    }

    public void armCurrent() {
        armCurrent = arm.getCurrentPosition();
    }






    public void resetOutArm() {outArm.resetEncoder();}
    public void resetArm() {arm.resetEncoder();}

    public String[] getArmTelemetry() {
        return new String[]{
                ("Current Arm Pos: " + String.valueOf(arm.getCurrentPosition())),
                ("Target Arm Pos: " + String.valueOf(ArmTarget)),
                ("Current Out Arm Pos: " + String.valueOf(outArm.getCurrentPosition())),
                ("Target Out Arm Pos: " + String.valueOf(OutTarget))

        };
    }


}