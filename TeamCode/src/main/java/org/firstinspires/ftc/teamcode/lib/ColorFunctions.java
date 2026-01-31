package org.firstinspires.ftc.teamcode.lib;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.Constants.COLORS;

@Config
public class ColorFunctions {
    public static double fRed=0.15,fBlue=0.35,fGreen=0.4,fTotal=900;
    public static COLORS getColor(int r, int g, int b) {
        float total = r + g + b;
        if (total < 1)
            total = 1;
        float normRed = r / total;
        float normGreen = g / total;
        float normBlue = b / total;

        if(total<fTotal) return COLORS.EMPTY;
        if (normRed > fRed && normBlue > fBlue && normGreen < 0.5)
            return COLORS.PURPLE;
        else if (normGreen > fGreen && normGreen > normBlue && normGreen > normRed)

            return COLORS.GREEN;
        else
            return COLORS.EMPTY;
    }

}

