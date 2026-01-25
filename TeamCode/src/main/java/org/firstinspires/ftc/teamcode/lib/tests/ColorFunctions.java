package org.firstinspires.ftc.teamcode.lib.tests;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.Constants.COLORS;

@Config
public class ColorFunctions {
    public static double fRed=0.2,fBlue=0.29,fGreen=0.5,fTotal=1500;
    public static COLORS getColor(int r, int g, int b) {
        float total = r + g + b;
        if (total < 1)
            total = 1;
        float normRed = r / total;
        float normGreen = g / total;
        float normBlue = b / total;

        if (normRed > fRed && normBlue > fBlue && normGreen < fGreen && total > fTotal)
            return COLORS.PURPLE;
        else if (normGreen > 0.5 && normGreen > normBlue && normGreen > normRed)

            return COLORS.GREEN;
        else
            return COLORS.EMPTY;
    }

}

