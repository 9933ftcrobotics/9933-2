package org.firstinspires.ftc.teamcode.subsystems;

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
        wrist.setInverted(false);
        wrist.setRange(MIN_ANGLE, MAX_ANGLE);
    }

    public void grabberPlace() { grabber.set(1); }

    public void grabberPick() { grabber.set(-1); }

    public void grabberStop() { grabber.set(0); }

    public void SetWristCenter() {
        wrist.setPosition(0.7);
    }

    public void SetWristLeft() {
        wrist.setPosition(0);
    }

    public void SetWristRight() {
        wrist.setPosition(1);
    }

}