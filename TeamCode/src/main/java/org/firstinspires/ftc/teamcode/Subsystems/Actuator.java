/*
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class Actuator implements Constants {

    public DcMotor motorActuator;
    private AutonomousOpMode auto;
    private Hardware hardware;
    PIDController climberControl = new PIDController(climberKP,climberKI,climberKD,climberMaxI);

    public double actuatorPower = motorActuator.getPower();
    public double actuatorPos = motorActuator.getCurrentPosition();

    public Actuator(Hardware hardware){
        this.motorActuator = hardware.motorActuator;
    }

    public void raise(){
        */
/*motorActuator.setPower(ACTUATOR_POWER);*//*

        double power = getPower(RAISE_POSITION);
        setPower(power);
    }

    public void lower(){
        */
/*motorActuator.setPower(-ACTUATOR_POWER);*//*

        double power = getPower(LOWER_POSITION);
        setPower(power);
    }

    public void stop(){
        motorActuator.setPower(0);
    }

    public double getPower(double position){
        long startTime = System.nanoTime();
        long stopState = 0;

        double ePos = (hardware.motorActuator.getCurrentPosition());
        double power = climberControl.power(position,ePos);

        if(Math.abs(position-ePos)<= ACTUATOR_TOLERANCE){
            stop();
        }

        return power;

    }

    public void setPower(double power){
        motorActuator.setPower(power);
    }

    public void reset(){
        motorActuator.setPower(0);
        motorActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorActuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double[] getData(){
        double aData[] = {actuatorPower,actuatorPos};
        return aData;
    }

    public boolean opModeIsActive()
    {
        return auto.getOpModeIsActive();
    }

}
*/
