package org.firstinspires.ftc.teamcode.Control;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Toggle {
    boolean boolState = true;
    boolean currState = false;
    boolean prevState = false;
    boolean task = true;
    private Gamepad gamepad;


    public boolean toggle(boolean boolState /*, boolean task*/){

        if(boolState){
            currState = true;
        }

        else{
            currState = false;
            if(prevState){
                task = !task;
            }
        }

        prevState = currState;

        return task;
    }










}
