import java.net.MalformedURLException;
import java.rmi.NotBoundException;
import java.rmi.RemoteException;

import lejos.remote.ev3.RemoteEV3;


public class VisualServoRobot 
{
	public static final String EV3_USB_ADDR = "10.0.1.1";
	
	public static void main(String [] args)
	{
		
	}
	
	public static class Robot
	{
		protected RemoteEV3 ev3;
		
		public Robot() throws RemoteException, MalformedURLException, NotBoundException
		{
			ev3 = new RemoteEV3(EV3_USB_ADDR);
			ev3.getAudio().systemSound(1);
		}
	}
}
