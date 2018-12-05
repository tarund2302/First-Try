package org.firstinspires.ftc.teamcode.Control;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Toggle {
    boolean boolState = true;
    boolean currState = false;
    boolean prevState = false;
    boolean taskState = true;
    private Gamepad gamepad;


    public boolean toggle(boolean boolState){

        if(boolState){
            currState = true;
        }

        else{
            currState = false;
            if(prevState){
                taskState = !taskState;
            }
        }

        prevState = currState;

        return taskState;
    }










}
