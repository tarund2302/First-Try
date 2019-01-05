package org.firstinspires.ftc.teamcode.Subsystems.RobotComponents;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class ChainLift implements Constants {

    public DcMotor hangMotor;
    public CRServo latch;

    private Telemetry telemetry;
    private AutonomousOpMode auto;
    private Hardware hardware;
    PIDController controlClimb = new PIDController(climberKP,climberKI);

    public ChainLift(Hardware hardware){
        this.hardware = hardware;
        this.hangMotor = hardware.hangMotor;
        this.latch = hardware.latch;
        this.telemetry = hardware.telemetry;
        this.auto = hardware.auto;
    }

    public void driverControl(Gamepad gamepad){
        if(gamepad.right_trigger > 0){
            raise();
        }
        if(gamepad.left_trigger > 0){
            lower();
        }
        else{
            liftStop();
        }

        if(gamepad.left_bumper){
            unlatch();
        }
        else if(gamepad.right_bumper){
            latch();
        }
        else{
            latchStop();
        }
    }

    public void raise(){
        hangMotor.setPower(1);
    }
    public void lower(){
        hangMotor.setPower(-1);
    }
    public void liftStop(){
        hangMotor.setPower(0);
    }

    public void latch(){
        latch.setPower(1);
    }
    public void unlatch(){
        latch.setPower(-1);
    }
    public void latchStop(){
        latch.setPower(0);
    }

    public void autoRaise(){
        double power = getPower(AUTO_RAISE_POSITION);
        setPower(power);
    }
    public void autoLower(){
        double power = getPower(AUTO_LOWER_POSITION);
        setPower(power);
    }
    public void reset(){
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Position:",getPos());
        telemetry.update();
    }
    public double getPos(){
       return hangMotor.getCurrentPosition();
    }

    public double getPower(double position){

        double ePos = hangMotor.getCurrentPosition();
        double power = controlClimb.power(position,ePos);

        if(Math.abs(position-ePos)<= HANG_TOLERANCE){
            liftStop();
        }
        return power;
    }
    public void setPower(double power){
        hangMotor.setPower(power);
    }
    public void setPowerForTime(double power, long time){
        long startTime = System.nanoTime();
        long stopState = 0;

        while(stopState <= time){
            setPower(power);
            stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
            telemetry.addData("Stop state",stopState);
            telemetry.update();
        }
        liftStop();
    }


}
