package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.abs;

import android.drm.DrmStore;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.DriveConstants;

public class ArmSubsystem extends SubsystemBase {
    //public static double upCurrent;
    private Motor arm;
    private Motor outArm;

    private boolean outArmInPos, upArmInPos = false;

    double outPower = 0;

    private PIDController controller;

    private double targetPos = 0;
    private double targetPos2 = 0;
    private double outTargetPos = 0;
    public static int outCurrent;
    public static int upCurrent;

    public static int target = 0;
    public static int outTarget = 0;

    //private final double ticks_in_degree =  5281.1 / 360; //gobilda 30rpm encoder
    private final double ticks_in_degree =  8192 / 360; //rev thru bore

    public static double p = 0.0025, i = 0, d = 0.0004;
    public static double f = 0.07;

    public ArmSubsystem(Motor arm, Motor outArm) {
        arm.setInverted(false);
        outArm.setInverted(true);
        this.arm = arm;
        this.outArm = outArm;
        arm.resetEncoder();
        outArm.resetEncoder();
        controller = new PIDController(p,i,d);
    }

    public void setArm(int Pos) {
        upCurrent = arm.getCurrentPosition();
        targetPos = Pos;
        // set the run mode

        PIDController pid = new PIDController(p,i,d);
        double ff = Math.cos(Math.toRadians(Pos / ticks_in_degree))*f;

        // set the tolerance
        double tolerence = 4;   // allowed maximum error
        // perform the control loop
        //if (Math.abs(Pos-arm.getCurrentPosition()) > tolerence) {
            arm.set(pid.calculate(arm.getCurrentPosition(),Pos) + ff);
            upArmInPos = false;
        /*} else {
            arm.stopMotor(); // stop the motor
            upArmInPos = true;
        }*/

        /*controller.setPID(p,i,d);
        int armPos = arm.getCurrentPosition();

        double pid = controller.calculate(armPos,target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree))*f;


        double power = pid + ff;

        arm.set(power);*/

    }

    public void setOutArm(int outPos) {

        outCurrent = outArm.getCurrentPosition();
        outTargetPos = outPos;
        // set the run mode

        PIDController pid = new PIDController(0.004,0,0);
        //double ff = Math.cos(Math.toRadians(outPos / ticks_in_degree))*f;

        // set the tolerance
        double tolerence = 13.6;   // allowed maximum error
        // perform the control loop
        if (Math.abs(outPos-outArm.getCurrentPosition()) > tolerence) {
            outPower = pid.calculate(outArm.getCurrentPosition(),outPos)/* + ff*/;
            if (outPower > 0.6) {
                outPower = 0.6;
            }
            outArm.set(outPower);
            upArmInPos = false;
        } else {
            outArm.stopMotor(); // stop the motor
            upArmInPos = true;
        }
    }

    public void powerArm(double Power) {
        arm.set(Power);
    }

    public void powerOutArm(double Power) {
        outArm.set(Power);
    }

    public void armCurrent() {
        upCurrent = arm.getCurrentPosition();
    }

    public void stopUpDown() {
        arm.stopMotor();
    }

    public void stopInOut() {
        outArm.stopMotor();
    }



    /*public Action Sleep(int Sec) {
        wait(1);

        return upOutAuto(Pos);
    }*/

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
            setArm(DriveConstants.armSamplePick + 65);
            Run = Run + 1;
            return !upArmInPos && Run < 30;
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
            setArm(DriveConstants.armSampleScoreHigh);
            Run = Run + 1;
            return !upArmInPos && Run < 50; //Time off?
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
            setArm(DriveConstants.armSpecimenClip - 300);
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
            return !outArmInPos && Run < 50;
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
            return !outArmInPos && Run < 25;
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
            return !outArmInPos && Run < 50;
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
            return !outArmInPos && Run < 85;
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
            setOutArm(600);
            Run = Run + 1;
            return !outArmInPos && Run < 30;
        }
    }
    public Action outPickFar() {
        return new OutPickFar();
    }

    public class UpPickFar implements Action {
        int Run = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setArm(DriveConstants.armSamplePickFar);
            Run = Run + 1;
            return !outArmInPos && Run < 30;
        }
    }
    public Action upPickFar() {
        return new UpPickFar();
    }




    public class OutMid implements Action {
        int Run = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setArm(DriveConstants.armOutSampleScoreHigh / 2);
            Run = Run + 1;
            return !outArmInPos && Run < 50;
        }
    }
    public Action outMid() {
        return new OutMid();
    }

}