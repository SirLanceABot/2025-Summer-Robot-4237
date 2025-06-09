package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.lang.invoke.MethodHandles;
import java.util.Map;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

/**
 * LEDs :)
 * @author Greta
 * @author Niyati
 */
public class LEDs extends SubsystemLance
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }
    

    // *** INNER ENUMS and INNER CLASSES ***
    // Put all inner enums and inner classes here

    public enum ColorPattern
    {
        kSolid,
        kBlink,
        kBreathe,
        kGradient,
        kRainbow,
        kOff;
    }
    // public enum Color
    // {
    //     kRed(255, 0, 0),
    //     kGreen(0, 255, 0),
    //     kBlue(0, 0, 255),
    //     kYellow(255, 255, 0),
    //     kPurple(255, 0, 255),
    //     kHotPink(255, 105, 180),
    //     kOff(0, 0, 0);

    //     int r;
    //     int g;
    //     int b;

    //     private Color(int r, int g, int b)
    //     {
    //         this.r = r;
    //         this.g = g;
    //         this.b = b;
    //     }

    // }


    
    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instance variables here
    private LEDPattern gradient;
    
    private final AddressableLED led = new AddressableLED(Constants.LEDs.LED_PORT);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(Constants.LEDs.LED_LENGTH);
    // private final AddressableLEDBuffer setBuffer;
    // private final AddressableLEDBuffer greenBuffer = new AddressableLEDBuffer(60);
    // private final AddressableLEDBuffer blueBuffer = new AddressableLEDBuffer(60);
    // private final AddressableLEDBuffer yellowBuffer = new AddressableLEDBuffer(60);
    // private final AddressableLEDBuffer purpleBuffer = new AddressableLEDBuffer(60);
    // private final Timer timer = new Timer();
    // private final LEDPattern rainbow = LEDPattern.rainbow(255, 100);
    // private static final Distance LEDSpacing = Meters.of(1/ 120.0);
    private LEDPattern solid;
    private final LEDPattern rainbow = LEDPattern.rainbow(255, 255);
    private LEDPattern off = LEDPattern.solid(Color.kBlack);

    private LEDPattern base;
    private LEDPattern breathePattern;
    private LEDPattern blinkPattern;

    private LEDPattern base2 =  LEDPattern.rainbow(255, 255);
    private Map<Double, Color> maskSteps = Map.of(0.5, Color.kWhite, 0.8, Color.kBlack);
    private LEDPattern mask ;
    private LEDPattern awesomePattern;

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new LEDs. 
     */
    public LEDs()
    {
        super("LEDs");
        System.out.println("  Constructor Started:  " + fullClassName);
        
        // timer.start();
        
        led.setLength(Constants.LEDs.LED_LENGTH);
        // blankBuffer = new AddressableLEDBuffer(5);
        // setBuffer = new AddressableLEDBuffer(5);

        // configBuffers();
        configLEDs();
        System.out.println("  Constructor Finished: " + fullClassName);
    }

    private void configLEDs()
    {
        // led.setData(ledBuffer);
        led.start();
    }

    // private void configBuffers()
    // {
    //     for(int i = 0; i < ledBuffer.getLength(); i++)
    //     {
    //         ledBuffer.setRGB(i, 0, 0, 0);
    //     }
    // }

    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    /**
     * Turns LEDs off
     */
    private void off()
    {
        // led.setData(ledBuffer);
        // setColorSolidCommand(Color.kOff);
        off.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }

    /**
     * Makes LEDs a solid color
     * @param color
     */
    private void setColorSolid(Color color)
    {
        solid = LEDPattern.solid(color);
        solid.applyTo(ledBuffer);
    }

    /**
     * Makes LEDs a solid color
     * @param color
     * @param brightness (int)
     */
    private void setColorSolid(int brightness, Color color)
    {
        solid = LEDPattern.solid(color).atBrightness(Percent.of(brightness));
        solid.applyTo(ledBuffer);
    }

    // private void setColorBlink(int r, int g, int b)
    // {
    //     for(int i = 0; i < ledBuffer.getLength(); i++)
    //     {
    //         setBuffer.setRGB(i, r, g, b);
    //     }

    //     if(timer.get() % 0.4 < 0.2)
    //     {
    //         led.setData(setBuffer);
    //     }
    //     else
    //     {
    //         led.setData(ledBuffer);
    //     }
    // }

    /**
     * Turns LEDs rainbow
     */
    private void setColorRainbow()
    {
        rainbow.applyTo(ledBuffer);
    }

    /**
     * Creates gradient with specified colors
     * @param colors
     */
    private void setColorGradient(Color... colors)
    {
        gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, colors);
        gradient.applyTo(ledBuffer);
    }

    /**
     * Creates gradient with specified colors
     * @param colors
     * @param brightness (int)
     */
    private void setColorGradient(int brightness, Color ... colors)
    {
        gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, colors);
        gradient.applyTo(ledBuffer);
    }

    /**
     * Creates a gradient or solid color
     * Blinks slowly - like breathing
     * @param colors
     */
    private void setColorBreathe(Color... colors)
    {
        base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, colors);
        breathePattern = base.breathe(Units.Seconds.of(2));
        breathePattern.applyTo(ledBuffer);
    }

    /**
     * Creates a gradient or solid color
     * Blinks slowly - like breathing
     * @param colors
     * @param brightness (int)
     */
    private void setColorBreathe(int brightness, Color ... colors)
    {
        base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, colors);
        breathePattern = base.breathe(Units.Seconds.of(2));
        breathePattern.applyTo(ledBuffer);
    }

    /**
     * Creates a gradient or solid blink pattern
     * @param colors
     */
    private void setColorBlink(Color... colors)
    {
        base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, colors);
        blinkPattern = base.breathe(Units.Seconds.of(0.5));
        blinkPattern.applyTo(ledBuffer);
    }

    /**
     * Creates a gradient or solid blink pattern
     * @param colors
     * @param brightness
     */
    private void setColorBlink(int brightness, Color ... colors)
    {
        base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, colors);
        blinkPattern = base.breathe(Units.Seconds.of(0.5));
        blinkPattern.applyTo(ledBuffer);
    }

    private void setEpilepticAttack()
    {
        mask = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(300));
        awesomePattern = base2.mask(mask);
        awesomePattern.applyTo(ledBuffer);
    }

    //COMMANDS

    public Command offCommand()
    {
        return runOnce(() -> off()).withName("Turn Off LED");
    }

    public Command setColorSolidCommand(Color color)
    {
        return runOnce(() -> setColorSolid(color)).withName("Set LED Solid");
    }

    public Command setColorSolidCommand(int brightness, Color color)
    {
        return runOnce(() -> setColorSolid(brightness, color)).withName("Set LED Solid");
    }

    public Command setColorGradientCommand(Color... colors)
    {
        return runOnce(() -> setColorGradient(colors)).withName("Set LED Gradient");
    }

    public Command setColorGradientCommand(int brightness, Color colors)
    {
        return runOnce(() -> setColorGradient(brightness, colors)).withName("Set LED Gradient");
    }

    public Command setColorRainbowCommand()
    {
        return run(() -> setColorRainbow()).withName("Set LED Rainbow");
    }

    public Command setColorBreatheCommand(Color... colors)
    {
        return run(() -> setColorBreathe(colors)).withName("Set LED Breathe");
    }

    public Command setColorBreatheCommand(int brightness, Color ... colors)
    {
        return run(() -> setColorBreathe(brightness, colors)).withName("Set LED Breathe");
    }

    public Command setColorBlinkCommand(Color... colors)
    {
        return run(() -> setColorBlink(colors)).withName("Set LED Blink");
    }

    public Command setColorBlinkCommand(int brightness, Color... colors)
    {
        return run(() -> setColorBlink(brightness, colors)).withName("Set LED Blink");
    }

    public Command setEpilepticAttackCommand()
    {
        return run(() -> setEpilepticAttack()).withName("Sets LEDs to give someone an Epileptic Attack");
    }
    
    // Use a method reference instead of this method
    // public Command stopCommand()
    // {
    //     return run( () -> stop() );
    // }


    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    @Override
    public void periodic()
    {
        
        led.setData(ledBuffer);
        // This method will be called once per scheduler run
        // Use this for sensors that need to be read periodically.
        // Use this for data that needs to be logged.
    }

    @Override
    public String toString()
    {
        return "";
    }
}
