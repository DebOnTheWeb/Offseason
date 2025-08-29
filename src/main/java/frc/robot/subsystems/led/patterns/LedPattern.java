package frc.robot.subsystems.led.patterns;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.led.ZonedAddressableLEDBuffer;


public abstract class LedPattern {
    private final boolean isDynamic;

    protected LedPattern(boolean isDynamic) {
        this.isDynamic = isDynamic;
    }

    public final boolean isDynamic() {
        return isDynamic;
    }

    public abstract void apply(ZonedAddressableLEDBuffer buffer);

    public void setLength(int length) {}

    protected static int[] getHSV(Color color) {
        float[] val = java.awt.Color.RGBtoHSB((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), null);
        return new int[]{
                (int) (val[0] * 180),
                (int)(val[1] * 255),
                (int)(val[2] * 255)};
    }
}
