package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class Actuator implements Constants {

    public DcMotor motorActuator;

    public double actuatorData = motorActuator.getPower();

    public Actuator(Hardware hardware){
        this.motorActuator = hardware.motorActuator;
    }

    public void raise(){
        motorActuator.setPower(ACTUATOR_POWER);
    }

    public void lower(){
        motorActuator.setPower(-ACTUATOR_POWER);
    }

    public void stop(){
        motorActuator.setPower(0);
    }

    public double getData(){
        return actuatorData;
    }
}
