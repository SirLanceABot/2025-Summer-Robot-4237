package frc.robot.sensors;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;


import java.lang.invoke.MethodHandles;

public class LaserCanSensor extends SensorLance
{
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();
    private LaserCan laserCAN = new LaserCan(0);
    
    // configures default settings for sensor from grapple
    public void configureLaserCAN()
    {
        try
        {
            laserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCAN.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            laserCAN.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } 
        catch (ConfigurationFailedException e)
        {
            System.out.println("Configuration error detected:  " + e);
        }
    }

    public LaserCanSensor()
    {
        super("really cool laser CAN");
        System.out.println("Constructor Started:  " + fullClassName );


        System.out.println("Constructor Started:  " + fullClassName );

    }

    @Override
    public void periodic()
    {
        // LaserCan.Measurement measurement = laserCAN.getMeasurement();
        // if(measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
        // {
        //     System.out.println("The target is " + measurement.distance_mm + " mm away");
        // }        
        // else
        // {
        //     System.out.println("Target out of range");
        // }

        // these statements are also in RobbieF test

    }

    @Override
    public String toString()
    {
        return "really cool laser CAN";
    }
}
