import java.net.MalformedURLException;
import java.rmi.NotBoundException;
import java.rmi.RemoteException;

import lejos.remote.ev3.RMIRegulatedMotor;
import lejos.remote.ev3.RMISampleProvider;
import lejos.remote.ev3.RemoteEV3;

public class Demo
{
    public static TrackerReader tracker;
    public static Robot robot;

    public static void main (String[] args)
    {
        try
        {
        	robot = new Robot();
        }
        catch(Exception e)
        {
        	System.err.println("Couldn't connect to the EV3: " + e.getMessage());
        	return;
        }
        
        robot.rotateMotorA( 200);
        robot.rotateMotorB(-200);
        
        for (float f : robot.getSampleA())
        {
        	System.out.println(String.format("%f ", f));
        }
        
        robot.close();
    }
    
    public static class Robot
    {
    	public static final String USB_ADDR = "10.0.1.1";
    	public static final int MOTOR_SPEED_DEG_PER_SECONDS = 10;
    	
    	protected RemoteEV3 robot;
    	protected RMIRegulatedMotor motorA;
    	protected RMIRegulatedMotor motorB;
    	protected RMISampleProvider sensorA;
    	
    	public Robot()
    			throws RemoteException, MalformedURLException, NotBoundException
    	{
            System.out.println("Connecting to EV3...");
            
    		robot = new RemoteEV3(USB_ADDR);
    		
            System.out.println("EV3 connected.");
    		
    		robot.getAudio().systemSound(2);
    		
    		System.out.println("Connecting motors...");
    		try
    		{
        		motorA = robot.createRegulatedMotor("A", 'L');
        		motorA.flt(true);
    		}
    		catch(Exception e)
    		{
    			System.err.println("Couldn't connect motor A: " + e.getMessage());
    		}
    		try
    		{
        		motorB = robot.createRegulatedMotor("B", 'L');
        		motorB.flt(true);
    		}
    		catch(Exception e)
    		{
    			System.err.println("Couldn't connect motor A: " + e.getMessage());
    		}
    		System.out.println("Motors connected.");
    		
    		System.out.println("Connecting sensors");
    		try
    		{
    			sensorA = robot.createSampleProvider("S1", "lejos.hardware.sensor.NXTColorSensor", "RGB");
    		}
    		catch(Exception e)
    		{
    			System.err.println("Couldn't connect sensor A: " + e.getMessage());
    		}
    		
    		System.out.println("EV3 setup finished.");
    	}
    	
    	public void rotateMotorA(int angle)
    	{
    		try
			{
				motorA.rotate(angle);
			} 
    		catch (RemoteException e)
			{
				System.err.println("Unable to rotate motor A: " + e.getMessage());
			}
    	}
    	
    	public void rotateMotorB(int angle)
    	{
    		try
			{
				motorB.rotate(angle);
			} 
    		catch (RemoteException e)
			{
				System.err.println("Unable to rotate motor B: " + e.getMessage());
			}
    	}
    	
    	public float [] getSampleA()
    	{
    		try
    		{
    			return sensorA.fetchSample();
    		}
    		catch (RemoteException e)
    		{
    			System.err.println("Unable to read from sensor A:" + e.getMessage());
    			return new float[] { };
    		}
    	}
    	
    	public void close()
    	{
    		try
			{
				motorA.close();
			} 
    		catch (RemoteException e)
			{
				System.err.println("Unable to close motor A: " + e.getMessage());
			}
    		
    		try
    		{
	    		motorB.close();
    		}
    		catch (RemoteException e)
    		{
				System.err.println("Unable to close motor B: " + e.getMessage());
    		}
    		
    		try
    		{
    			sensorA.close();
    		}
    		catch (RemoteException e)
    		{
    			System.err.println("Unable to close sensor A: " + e.getMessage());
    		}
    		
    		robot.getAudio().systemSound(3);
    		System.out.println("Remote EV3 connection closed.");
    	}
    }
}
