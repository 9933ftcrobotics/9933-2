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
    private Motor arm;
    private Motor outArm;

    private boolean outArmInPos, upArmInPos = false;

    private PIDController controller;

    private double targetPos = 0;
    private double targetPos2 = 0;
    private double outTargetPos = 0;
    public static int outCurrent;
    public static int upCurrent;

    //public static double f = 1;

    public static int target = 0;
    public static int outTarget = 0;

    private final double ticks_in_degree =  5281.1 / 360;

    public static double p = 0.05, i = 0, d = 0;
    //public static double f = 2;

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

        PIDController pid = new PIDController(0.008,0,1);
        //double ff = Math.cos(Math.toRadians(Pos / ticks_in_degree))*f;

        // set the tolerance
        double tolerence = 1;   // allowed maximum error
        // perform the control loop
        if (Math.abs(Pos-arm.getCurrentPosition()) > tolerence) {
            arm.set(pid.calculate(arm.getCurrentPosition(),Pos)/* + ff*/);
            upArmInPos = false;
        } else {
            arm.stopMotor(); // stop the motor
            upArmInPos = true;
        }

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

        PIDController pid = new PIDController(0.005,0,0);
        //double ff = Math.cos(Math.toRadians(outPos / ticks_in_degree))*f;

        // set the tolerance
        double tolerence = 13.6;   // allowed maximum error
        // perform the control loop
        if (Math.abs(outPos-outArm.getCurrentPosition()) > tolerence) {
            outArm.set(pid.calculate(outArm.getCurrentPosition(),outPos)/* + ff*/);
            upArmInPos = false;
        } else {
            outArm.stopMotor(); // stop the motor
            upArmInPos = true;
        }
    }

    public void powerArm(double Power) {
        arm.set(Power);
    }

    public void armCurrent() {
        DriveConstants.armCurrent = arm.getCurrentPosition();
    }

    public Action upArmAuto(int Pos) {
        PIDController pid = new PIDController(0.008,0,1);
        //double ff = Math.cos(Math.toRadians(Pos / ticks_in_degree))*f;

        // set the tolerance
        double tolerence = 1;   // allowed maximum error
        // perform the control loop
        if (Math.abs(Pos-arm.getCurrentPosition()) > tolerence) {
            arm.set(pid.calculate(arm.getCurrentPosition(),Pos)/* + ff*/);
            outArmInPos = false;
        } else {
            arm.stopMotor(); // stop the motor
            outArmInPos = true;
        }

        return upArmAuto(Pos);
    }

    public Action upOutAuto(int Pos) {
        PIDController pid = new PIDController(0.005,0,1);
        //double ff = Math.cos(Math.toRadians(Pos / ticks_in_degree))*f;

        // set the tolerance
        double tolerence = 1;   // allowed maximum error
        // perform the control loop
        if (Math.abs(Pos-outArm.getCurrentPosition()) > tolerence) {
            outArm.set(pid.calculate(outArm.getCurrentPosition(),Pos)/* + ff*/);
        } else {
            outArm.stopMotor(); // stop the motor
        }

        return upOutAuto(Pos);
    }

    /*public Action Sleep(int Sec) {
        wait(1);

        return upOutAuto(Pos);
    }*/

    public void resetOutArm() {outArm.resetEncoder();}

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
            return !upArmInPos && Run < 10;
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
            return !upArmInPos && Run < 6;
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
            return !upArmInPos && Run < 6;
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
            return !upArmInPos && Run < 6;
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
            setArm(DriveConstants.armSpecimenClip - 175);
            run = run + 1;
            return !upArmInPos && run < 5;
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
            return !outArmInPos && Run < 10;
        }
    }
    public Action outRest() {
        return new OutRest();
    }

    public class OutPick implements Action {
        int Run = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setOutArm(DriveConstants.armOutSamplePick);
            Run = Run + 1;
            return !outArmInPos && Run < 6;
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
            return !outArmInPos && Run < 6;
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
            return !outArmInPos && Run < 6;
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
            return !outArmInPos && Run < 15;
        }
    }
    public Action outSpecimenScore() {
        return new OutSpecimenScore();
    }

}