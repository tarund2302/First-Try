package org.firstinspires.ftc.teamcode.OpModes.AutoOpModes;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.GoldFinder;

//v2 auto code made by Tarun Dasari, Rookie Programmer of 3846 Maelstrom
//last update: November 21, 2018

@Autonomous(name = "AutoOp")
public class AutoOp_TestSensors extends LinearOpMode implements Constants, AutonomousOpMode
{

    private Hardware robot = new Hardware();

    public boolean getOpModeIsActive()
    {
        return opModeIsActive();
    }

    public Telemetry getTelemetry()
    {
        return telemetry;
    }


    @Override
    public void runOpMode() {

        robot.setAuto(this);
        robot.setTelemetry(telemetry);
        robot.init(hardwareMap);

        GoldFinder gold = new GoldFinder(this, robot);
        gold.setAlignSettings(ALIGN_POSITION,1000);
        double goldPos = 0;

        waitForStart();

        robot.drivetrain.eReset();

        //test

        //testing for gyro
        robot.drivetrain.turnAngle(40);
        robot.drivetrain.stopTime(2500);
        robot.drivetrain.turnAngle(-35);
        robot.drivetrain.stopTime(5000);
        robot.drivetrain.turnAngle(20);
        robot.drivetrain.stopTime(5000);
        robot.drivetrain.turnAngle(30);
        robot.drivetrain.stop();
   /*
        //testing one side turns
        robot.drivetrain.sideTurnAngle(30, true);
        robot.drivetrain.stopTime(2500);
        robot.drivetrain.sideTurnAngle(-25, false);
        robot.drivetrain.stopTime(3000);
        robot.drivetrain.sideTurnAngle(90, true);
        robot.drivetrain.stopTime(2000);;
        robot.drivetrain.sideTurnAngle(30, false);
        robot.drivetrain.stopTime(2500);
        robot.drivetrain.sideTurnAngle(100, false);
        robot.drivetrain.stop();
        */
   /*     //testing for encoders
        robot.drivetrain.driveDistance(24);
        robot.drivetrain.stopTime(5000);
        robot.drivetrain.driveDistance(-12);
        robot.drivetrain.stopTime(300);
        robot.drivetrain.driveDistance(10);
        robot.drivetrain.stopTime(3000);
        robot.drivetrain.driveDistance(-10);
*/

/*
        //testing for range sensor
        robot.drivetrain.driveTillRangeDistance(4);
        robot.drivetrain.stopTime(5000);
        robot.drivetrain.driveTillRangeDistance(2);
        robot.drivetrain.stopTime(5000);
        robot.drivetrain.driveTillRangeDistance(6);
*/

 /*      gold.startOpenCV(hardwareMap);

        while(getOpModeIsActive() && !gold.isFound()){
            robot.drivetrain.rotate(-0.33);
            telemetry.addData("Aligned:", gold.getAligned());
            telemetry.addData("Pos:",gold.getXPosition());
            telemetry.update();
        }
        sleep(1000);
        robot.drivetrain.stop();
        gold.alignGold();
*/
    }




} //main

