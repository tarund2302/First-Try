package org.firstinspires.ftc.teamcode.Subsystems.AutoPaths;

import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Direction;
import org.firstinspires.ftc.teamcode.Control.GoldPos;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.RobotComponents.Drivetrain;

public class CraterPaths {

    GoldPos pos;
    public Hardware hardware;
    public Drivetrain dt = new Drivetrain(hardware);
    public AutonomousOpMode auto;

    public CraterPaths(AutonomousOpMode auto){
        this.auto = auto;
    }
    public CraterPaths(Hardware hardware){
        this.hardware = hardware;
    }

    public void run(){
        sample();
        driveToDepot();
        park();
    }

    private void sample(){
        if(pos == GoldPos.LEFT){
            dt.turnAngle(30, Direction.LEFT);
            dt.driveDistance(20,Direction.FORWARD);
            dt.driveDistance(10,Direction.BACKWARD);
        }
        else if(pos == GoldPos.MIDDLE){
            dt.driveDistance(15,Direction.FORWARD);
            dt.driveDistance(10,Direction.BACKWARD);
        }
        else if(pos == GoldPos.RIGHT){
            dt.turnAngle(30,Direction.RIGHT);
            dt.driveDistance(20,Direction.FORWARD);
            dt.driveDistance(10,Direction.BACKWARD);
        }
        else{
            dt.driveDistance(15,Direction.FORWARD);
            dt.driveDistance(10,Direction.BACKWARD);
        }
    }

    private void driveToDepot(){
        dt.turnAngle(90,Direction.LEFT);
        dt.driveDistance(60,Direction.FORWARD);
        dt.turnAngle(30,Direction.LEFT);
        dt.driveDistance(50,Direction.FORWARD);
    }

    private void park(){
        dt.driveDistance(70,Direction.BACKWARD);
    }


}
