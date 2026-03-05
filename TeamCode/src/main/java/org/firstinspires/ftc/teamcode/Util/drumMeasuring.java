package org.firstinspires.ftc.teamcode.Util;

public class drumMeasuring {
    int drumMeasuring(int drumFrom, int drumTo){
        if(Math.abs(drumFrom-drumTo)==2){
            return(600);
        }
        else if (Math.abs(drumFrom-drumTo)==1){
            return(300);
        }
        else{
            return(0);
        }
    }

}
