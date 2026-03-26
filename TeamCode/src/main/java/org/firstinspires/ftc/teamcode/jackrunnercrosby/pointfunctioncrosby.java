package org.firstinspires.ftc.teamcode.jackrunnercrosby;

public class pointfunctioncrosby {
    public double[][] linefunctioncrosby(double[] point1, double[] point2, int frequency){
        double[][] outputarray = new double[frequency][2];
        double xiteration = (point2[0]-point1[0])/frequency;
        double yiteration = (point2[1]-point1[1])/frequency;
        for (int i = 1; i<=outputarray.length; i++){
            outputarray[i][0] = point1[0] + xiteration*i;
            outputarray[i][1] = point1[1] + yiteration*i;
        }
        return outputarray;
    }
}
