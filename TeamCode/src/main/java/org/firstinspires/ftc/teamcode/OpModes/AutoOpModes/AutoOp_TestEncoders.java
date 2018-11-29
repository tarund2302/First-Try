package org.firstinspires.ftc.teamcode.OpModes.AutoOpModes;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

//v2 auto code made by Tarun Dasari, Rookie Programmer of 3846 Maelstrom
//last update: November 21, 2018

@Autonomous(name = "AutoOp_Encoders")
public class AutoOp_TestEncoders extends LinearOpMode implements Constants
{

    private Hardware robot = new Hardware();
    /*private ElapsedTime runtime = new ElapsedTime();*/
    //private Drivetrain drivetrain = new Drivetrain(Hardware hardwareMap);

    public boolean getOpModeIsActive()
    {
        return opModeIsActive();
    }

    public Telemetry getTelemetry()
    {
        return telemetry;
    }


    @Override
    public void runOpMode() throws InterruptedException {

        //robot.setAuto(this, telemetry);

        robot.init(hardwareMap);

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.drivetrain.eReset();

        /*robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/




        robot.hookServo.setPosition(HOOK_UP_POSITION);
        robot.markerServo.setPosition(MARKER_UP_POSITION);

        waitForStart();

        //unlatch
        robot.motorActuator.setPower(1);
        sleep(1260);
        robot.motorActuator.setPower(0);
        robot.hookServo.setPosition(HOOK_DOWN_POSITION);
        robot.motorActuator.setPower(-1);
        sleep(1500);
        robot.motorActuator.setPower(0);

        //drive towards crater
        robot.drivetrain.driveDistance(36);

        //extend marker lift and drop marker
        robot.motorExtendo1.setPower(EXTENDO_EXTEND_POWER);
        robot.motorExtendo2.setPower(EXTENDO_EXTEND_POWER);
        sleep(1000);
        robot.motorExtendo1.setPower(0);
        robot.motorExtendo2.setPower(0);
        robot.markerServo.setPosition(MARKER_DOWN_POSITION);
        sleep(500);
        robot.markerServo.setPosition(MARKER_UP_POSITION);

        //retract marker lift
        robot.motorExtendo1.setPower(EXTENDO_RETRACT_POWER);
        robot.motorExtendo2.setPower(EXTENDO_RETRACT_POWER);
        sleep(1000);
        robot.motorExtendo1.setPower(0);
        robot.motorExtendo2.setPower(0);

        //rotate towards crater from front of bot
        robot.drivetrain.turnAngle(-150);

        //drive towards crater
        robot.drivetrain.driveDistance(60);


        //extend to pass over crater rim
        robot.motorExtendo1.setPower(1);
        robot.motorExtendo2.setPower(1);
        sleep(1750);
        robot.motorExtendo1.setPower(0);
        robot.motorExtendo2.setPower(0);

        /*EncoderDrive(DRIVE_SPEED, 24, 24, 5.0);
        EncoderDrive(TURN_SPEED, -24, 24, 2.0);
        EncoderDrive(DRIVE_SPEED, 48, 48, 5.0);*/

        /*
        //testing for gyro
        robot.drivetrain.turnAngle(90);
        robot.drivetrain.turnAngle(-90);
        robot.drivetrain.stop();
        sleep(5000);
        robot.drivetrain.turnAngle(270);
        robot.drivetrain.turnAngle(-300);
        robot.drivetrain.stop();

        //testing for encoders
        robot.drivetrain.driveDistance(12);
        robot.drivetrain.driveDistance(-12);
        robot.drivetrain.driveDistance(40);
        robot.drivetrain.driveDistance(-40);*/

    }




} //main

