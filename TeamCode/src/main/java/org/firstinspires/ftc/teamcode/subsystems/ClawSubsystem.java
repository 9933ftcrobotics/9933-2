package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ClawSubsystem extends SubsystemBase {
    private CRServo grabber;
    private ServoEx wrist;

    public double MIN_ANGLE = 0;

    public double MAX_ANGLE = 1;
    public ClawSubsystem(CRServo grabber, ServoEx wrist) {
        this.grabber = grabber;
        this.wrist = wrist;
        // change the effective range to a min and max in DEGREES
        wrist.setInverted(true);
        wrist.setRange(MIN_ANGLE, MAX_ANGLE);
    }

    public void grabberPlace() { grabber.set(1); }
    public void grabberPlaceToPower(double Power) { grabber.set(Power); }

    public void grabberPick() { grabber.set(-1); }

    public void grabberStop() { grabber.set(0); }

    public Action grabberPlaceAuto() {
        grabber.set(1);
        return grabberPlaceAuto();
    }

    public Action grabberPickAuto() {
        grabber.set(-1);
        return grabberPickAuto();
    }

    public Action grabberStopAuto() {
        grabber.set(0);
        return grabberStopAuto();
    }



    public void SetWristCenter() {
        wrist.setPosition(0);
    }

    public void SetWristLeft() {
        wrist.setPosition(0);
    }

    public void SetWristRight() { wrist.setPosition(0.3); }


    public Action SetWristCenterAuto() {
        wrist.setPosition(0);
        return SetWristCenterAuto();
    }

    public Action SetWristRightAuto() {
        wrist.setPosition(0.3);
        return SetWristRightAuto();
    }

    public class WristRight implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            SetWristRight();
            return false;
        }
    }
    public Action rightWrist() {
        return new WristRight();
    }


    public class WristCenter implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            SetWristCenter();
            return false;
        }
    }
    public Action centerWrist() {
        return new WristCenter();
    }

    public class GrabberPick implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(-1);
            return false;
        }
    }
    public Action pickGrabber() {
        return new GrabberPick();
    }

    public class GrabberPlace implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            grabber.set(1);
            return false;
        }
    }
    public Action placeGrabber() {
        return new GrabberPlace();
    }


    public class GrabberStop implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            grabber.set(1);
            return false;
        }
    }
    public Action stopGrabber() {
        return new GrabberStop();
    }


}

