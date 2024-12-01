package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.DriveConstants;

public class ArmSubsystem_CC extends SubsystemBase {
    //public static double upCurrent;
    private Motor arm;
    private Motor outArm;

    private boolean outArmInPos, upArmInPos = false;

    double outPower = 0;

    private PIDController armPID,outPID;

    private double targetPos = 0;
    private double targetPos2 = 0;
    private double outTargetPos = 0;
    public static int outCurrent;
    public static int armCurrent;

    public static int ArmTarget = 0;
    public static int OutTarget = 0;

    //private final double ticks_in_degree =  5281.1 / 360; //gobilda 30rpm encoder
    private final double ticks_in_degree =  8192 / 360; //rev thru bore

    public static double arm_p = 0.0025, arm_i = 0, arm_d = 0.0004;
    public static double arm_f = 0.07;

    public static double out_p = 0.004, out_i = 0, out_d = 0;

    public ArmSubsystem_CC(Motor arm, Motor outArm) {
        arm.setInverted(false);
        outArm.setInverted(true);
        this.arm = arm;
        this.outArm = outArm;
        arm.resetEncoder();
        outArm.resetEncoder();
        armPID = new PIDController(arm_p,arm_i,arm_d);
        outPID = new PIDController(0.002,0,0);

    }

    public void setArms(int ArmTarget, int OutTarget) {

        armPID.setPID(arm_p,arm_i,arm_d);
        armCurrent = arm.getCurrentPosition();

        double armpid = armPID.calculate(armCurrent,ArmTarget);
        double armff = Math.cos(Math.toRadians(ArmTarget / ticks_in_degree))*arm_f;


        double power = armpid + armff;

        arm.set(power);

        outPID.setPID(out_p,out_i,out_d);
        outCurrent = outArm.getCurrentPosition();

        double outpid = outPID.calculate(outCurrent,OutTarget);

        outArm.set(outpid);


    }
    public void setArm(int ArmTarget) {

        armPID.setPID(arm_p, arm_i, arm_d);
        armCurrent = arm.getCurrentPosition();

        double armpid = armPID.calculate(armCurrent, ArmTarget);
        double armff = Math.cos(Math.toRadians(ArmTarget / ticks_in_degree)) * arm_f;


        double power = armpid + armff;

        arm.set(power);
    }

    public void setOutArm(int outPos) {
        outPID.setPID(out_p,out_i,out_d);
        outCurrent = outArm.getCurrentPosition();

        double outpid = outPID.calculate(outCurrent,OutTarget);

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
                ("Target Arm Pos: " + String.valueOf(targetPos)),
                ("Current Out Arm Pos: " + String.valueOf(outArm.getCurrentPosition())),
                ("Target Out Arm Pos: " + String.valueOf(outTargetPos))

        };
    }

    public class UpRest implements Action {
        int Run = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setArm(DriveConstants.armSampleRest);
            Run = Run + 1;
            return !upArmInPos && Run < 100;
        }
    }
    public Action upRest() {
        return new UpRest();
    }

    public class UpPick implements Action {
        int Run = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setArm(DriveConstants.armSamplePick);
            Run = Run + 1;
            return !upArmInPos && Run < 100;
        }
    }
    public Action upPick() {
        return new UpPick();
    }

    public class UpSpecimen implements Action {
        int Run = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //arm.setTargetPosition(DriveConstants.armSpecimenClip);
            setArm(DriveConstants.armSpecimenClip);
            Run = Run + 1;
            return !upArmInPos && Run < 125;
        }
    }
    public Action upSpecimen() {
        return new UpSpecimen();
    }

    public class UpHigh implements Action {
        int Run = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setArm(DriveConstants.armSampleScoreHigh + 25);
            Run = Run + 1;
            return !upArmInPos && Run < 125; //Time off?
        }
    }
    public Action upHigh() {
        return new UpHigh();
    }


    public class UpSpecimenScore implements Action {
        int run = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //arm.setTargetPosition(DriveConstants.armSpecimenRest);
            setArm(DriveConstants.armSpecimenClip - 200);
            run = run + 1;
            return !upArmInPos && run < 75;
        }
    }
    public Action upSpecimenScore() {
        return new UpSpecimenScore();
    }





    public class OutRest implements Action {
        int Run = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setOutArm(DriveConstants.armOutSampleRest);
            Run = Run + 1;
            return !outArmInPos && Run < 100;
        }
    }
    public Action outRest() {
        return new OutRest();
    }

    public class OutPick implements Action {
        int Run = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setOutArm(DriveConstants.armOutSamplePick + 50);
            Run = Run + 1;
            return !outArmInPos && Run < 100;
        }
    }
    public Action outPick() {
        return new OutPick();
    }

    public class OutSpecimen implements Action {
        int Run = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setOutArm(DriveConstants.armOutSpecimenClip);
            Run = Run + 1;
            return !outArmInPos && Run < 75;
        }
    }
    public Action outSpecimen() {
        return new OutSpecimen();
    }

    public class OutHigh implements Action {
        int Run = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setOutArm(DriveConstants.armOutSampleScoreHigh);
            Run = Run + 1;
            return !outArmInPos && Run < 130;
        }
    }
    public Action outHigh() {
        return new OutHigh();
    }


    public class OutSpecimenScore implements Action {
        int Run = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setOutArm(DriveConstants.armOutSpecimenRest);
            Run = Run + 1;
            return !outArmInPos && Run < 75;
        }
    }
    public Action outSpecimenScore() {
        return new OutSpecimenScore();
    }

    public class UpSeven implements Action {
        int Run = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setArm(700);
            Run = Run + 1;
            return !outArmInPos && Run < 50;
        }
    }
    public Action upTest() {
        return new UpSeven();
    }


    public class UpZero implements Action {
        int Run = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setArm(0);
            Run = Run + 1;
            return !outArmInPos && Run < 50;
        }
    }
    public Action upZero() {
        return new UpZero();
    }


    public class OutZero implements Action {
        int Run = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setOutArm(0);
            Run = Run + 1;
            return !outArmInPos && Run < 50;
        }
    }
    public Action outZero() {
        return new OutZero();
    }


    public class OutPickFar implements Action {
        int Run = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setOutArm(DriveConstants.armOutSamplePickFar);
            Run = Run + 1;
            return !outArmInPos && Run < 50;
        }
    }
    public Action outPickFar() {
        return new OutPickFar();
    }

    public class UpPickFar implements Action {
        int Run = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setOutArm(DriveConstants.armSamplePickFar);
            Run = Run + 1;
            return !outArmInPos && Run < 50;
        }
    }
    public Action upPickFar() {
        return new UpPickFar();
    }

}