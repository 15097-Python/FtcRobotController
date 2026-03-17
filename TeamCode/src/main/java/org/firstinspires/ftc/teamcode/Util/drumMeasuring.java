package org.firstinspires.ftc.teamcode.Util;

import static org.firstinspires.ftc.teamcode.Util.Enum.DrumSlots.AllSlots;
import static org.firstinspires.ftc.teamcode.Util.Enum.DrumSlots.SLOT_0;
import static org.firstinspires.ftc.teamcode.Util.Enum.DrumSlots.SLOT_1;
import static org.firstinspires.ftc.teamcode.Util.Enum.DrumSlots.SLOT_2;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Util.Enum.DrumSlots;

@Config
public class drumMeasuring {
    public static int longtime = 600;
    public static int shortime = 300;
    public static int drumMeasuringFunct(int drumFrom, int drumTo){
        DrumSlots fromSlot = intToSlot(drumFrom);
        DrumSlots toSlot = intToSlot(drumTo);
        if(fromSlot.shootPosition == .42 || toSlot.shootPosition == .42) return (shortime);
        return(longtime);
    }

    public static int currentAngleToSlot(double currentangle){
        if(currentangle == SLOT_0.shootPosition || currentangle == SLOT_0.loadPosition){
            return(0);
        } else if(currentangle == SLOT_1.shootPosition || currentangle == SLOT_1.loadPosition){
            return(1);
        }else if(currentangle == DrumSlots.SLOT_2.shootPosition || currentangle == DrumSlots.SLOT_2.loadPosition){
            return(2);
        }else{
            return(-1);
        }
    }

    public static DrumSlots intToSlot(int slotasint){
        switch (slotasint){
            case 0:
                return(SLOT_0);
            case 1:
                return(SLOT_1);
            case 2:
                return(SLOT_2);
        }
        return(AllSlots);
    }
}
