package org.firstinspires.ftc.teamcode.Subsystems.AutoPaths;

import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Direction;
import org.firstinspires.ftc.teamcode.Control.GoldPos;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.RobotComponents.Drivetrain;

public class DepotPaths {
    GoldPos pos;
    public Hardware hardware;
    public Drivetrain dt = new Drivetrain(hardware);
    public AutonomousOpMode auto;

    public DepotPaths(AutonomousOpMode auto){
        this.auto = auto;
    }
    public DepotPaths(Hardware hardware){
        this.hardware = hardware;
    }

    public void run(){
        driveToDepot();
        turnToCrater();
        park();
    }

    private void driveToDepot(){
        if(pos == GoldPos.LEFT){
            dt.turnAngle(30, Direction.LEFT);
            dt.driveDistance(50,Direction.FORWARD);
            dt.turnAngle(90, Direction.RIGHT);
            dt.driveDistance(50,Direction.FORWARD);

        }
        else if(pos == GoldPos.MIDDLE){
            dt.driveDistance(60,Direction.FORWARD);
        }
        else if(pos == GoldPos.RIGHT){
            dt.turnAngle(30,Direction.RIGHT);
            dt.driveDistance(57,Direction.FORWARD);
            dt.turnAngle(30,Direction.LEFT);
            dt.driveDistance(10,Direction.FORWARD);
        }
        else{
            dt.driveDistance(60,Direction.FORWARD);
        }
    }
    private void turnToCrater(){
        if(pos == GoldPos.LEFT){
            dt.turnAngle(90, Direction.RIGHT);
        }
        else if(pos == GoldPos.MIDDLE){
            dt.turnAngle(30,Direction.RIGHT);
        }
        else if(pos == GoldPos.RIGHT){
            dt.turnAngle(90,Direction.RIGHT);
        }
        else{
            dt.turnAngle(30,Direction.RIGHT);
        }
    }
    private void park(){
        if(pos == GoldPos.LEFT){
            dt.driveDistance(60,Direction.BACKWARD);
        }
        else if(pos == GoldPos.MIDDLE){
            dt.driveDistance(70,Direction.BACKWARD);
        }
        else if(pos == GoldPos.RIGHT){
            dt.driveDistance(70,Direction.BACKWARD);
        }
        else{
            dt.driveDistance(70,Direction.BACKWARD);
        }
    }
}
