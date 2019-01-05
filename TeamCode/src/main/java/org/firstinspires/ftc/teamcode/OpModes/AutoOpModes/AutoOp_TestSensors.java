package org.firstinspires.ftc.teamcode.OpModes.AutoOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.Direction;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.AutoPaths.DepotPaths;
import org.firstinspires.ftc.teamcode.Subsystems.RobotComponents.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.RobotComponents.GoldFinder;

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

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        robot.setAuto(this);
        robot.setTelemetry(telemetry);
        robot.init(hardwareMap);

        /*GoldFinder gold = new GoldFinder(this, robot);*/
        robot.gold.setAlignSettings(ALIGN_POSITION,1000);
        double goldPos = 0;

        waitForStart();
        runtime.reset();
        robot.drivetrain.eReset();

        //test

        //testing for gyro
        /*robot.drivetrain.turnAngle(40,Direction.LEFT);
        robot.drivetrain.stopTime(2500);
        robot.drivetrain.turnAngle(35,Direction.RIGHT);
        robot.drivetrain.stopTime(5000);
        robot.drivetrain.turnAngle(20,Direction.LEFT);
        robot.drivetrain.stopTime(5000);
        robot.drivetrain.turnAngle(30,Direction.LEFT);
        robot.drivetrain.stop();
*/
        /* //testing angle correction
        robot.drivetrain.testAngleCorrection();
        */

/*
        //testing one side turns
        robot.drivetrain.sideTurnAngle(30, "left");
        robot.drivetrain.stopTime(2500);
        robot.drivetrain.sideTurnAngle(-25, "right");
        robot.drivetrain.stopTime(3000);
        robot.drivetrain.sideTurnAngle(90, "left");
        robot.drivetrain.stopTime(2000);;
        robot.drivetrain.sideTurnAngle(30, "right");
        robot.drivetrain.stopTime(2500);
        robot.drivetrain.sideTurnAngle(100, "right");
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

        //testing directional drive
        robot.drivetrain.driveTest(24,Direction.FORWARD);
        robot.drivetrain.stopTime(5000);
        robot.drivetrain.driveTest(12,Direction.BACKWARD);
        robot.drivetrain.stopTime(300);
        robot.drivetrain.driveTest(10,Direction.FORWARD);
        robot.drivetrain.stopTime(3000);
        robot.drivetrain.driveTest(10,Direction.BACKWARD);
*/

/*
        //testing for range sensor
        robot.drivetrain.driveTillRangeDistance(4);
        robot.drivetrain.stopTime(5000);
        robot.drivetrain.driveTillRangeDistance(2);
        robot.drivetrain.stopTime(5000);
        robot.drivetrain.driveTillRangeDistance(6);
*/
/*

        robot.gold.startOpenCV(hardwareMap);

        */
/*      while(getOpModeIsActive() && !gold.isFound()){
            robot.drivetrain.rotate(-0.33);
            telemetry.addData("Aligned:", gold.getAligned());
            telemetry.addData("Pos:",gold.getXPosition());
            telemetry.update();
        }*//*

        robot.gold.goldSearch(-0.33);
        sleep(1000);
        robot.drivetrain.stop();
        robot.gold.alignGold();
*/
        //testing elapsed time
 /*       robot.drivetrain.driveForTime(0.25,15000);
        while(getOpModeIsActive() && !(runtime.seconds() > 30)){
            if(runtime.seconds() >= 10){
                robot.drivetrain.rotate(.75);
            }
        }
*/
    }


} //main

