package org.firstinspires.ftc.teamcode.Example;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class ConceptBlinkin
{
    RevBlinkinLedDriver blinkinLedDriverLeft;
    RevBlinkinLedDriver blinkinLedDriverRight;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    public void SetLeftToColor(int number, int side)
    {
        switch (number)
        {

            case 1:
                pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                break;
            case 2:
                pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                break;
            case 3:
                pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                break;
            case 4:
                pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                break;
            default:
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
                break;
        }
        if (side ==1)
        {
            blinkinLedDriverLeft.setPattern(pattern);
        }
        else
        {
            blinkinLedDriverRight.setPattern(pattern);
        }
    }


}
