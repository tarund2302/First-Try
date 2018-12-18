package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class Extender implements Constants {

    public DcMotor motorExtendo1;
    public DcMotor motorExtendo2;
    public Servo markerServo;

    public Extender(Hardware hardware){
        this.motorExtendo1 = hardware.motorExtendo1;
        this.motorExtendo2 = hardware.motorExtendo2;
        this.markerServo = hardware.markerServo;
    }

    public void extend(){
        motorExtendo1.setPower(EXTENDO_EXTEND_POWER);
        motorExtendo2.setPower(EXTENDO_EXTEND_POWER);
    }

    public void retract(){
        motorExtendo1.setPower(EXTENDO_RETRACT_POWER);
        motorExtendo2.setPower(EXTENDO_RETRACT_POWER);
    }

    public void stop(){
        motorExtendo1.setPower(0);
        motorExtendo2.setPower(0);
    }

    public void drop(){
        markerServo.setPosition(MARKER_DOWN_POSITION);
    }

    public void raise(){
        markerServo.setPosition(MARKER_UP_POSITION);
    }


}
