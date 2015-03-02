import lejos.utility.Matrix;


public class VisualServoRobot {
	
	
	private TrackerReader tracker;
	private Robot robot;
	
	private double maxGlobalIterations = 5.;
	private double maxLocalIterations = 30.;
	private double minError = 20;
	private int deltaAngle = 20;
	private double gain = 1;
	
	private Matrix mdf = null;
	public double x = 0;
	public double y = 0;
	public double targetX = 0;
	public double targetY = 0;
	//increase the performance...
	double[] globalErrors = null;
	
	public static void main(String [] args)
	{
		VisualServoRobot vsr = new VisualServoRobot();
		try
		{
			vsr.getInitialPosition();
			vsr.calculateJacobian(0,0);
			vsr.inverseNewtonWithInitialGuess(vsr.targetX,vsr.targetY);
			//vsr.inverseNewton(vsr.targetX,vsr.targetY,0,0);
		}
		catch(Exception e){}
		vsr.disconnect();
	}
	
	
	public VisualServoRobot(){
		try{ robot = new Robot();
		}catch(Exception e){
			System.err.println("Couldn't connect to the EV3: " + e.getMessage());
			return;
		}
		tracker = new TrackerReader();
		tracker.launchService(); //locking, until it's tracking an object
	}
	
	
	
	public void move(){
		robot.rotateMotorA(-400);
		robot.rotateMotorB(400);
	}
	
	public void disconnect(){
		robot.rotateToMotorA(0);
		robot.rotateToMotorB(0);
		robot.close();
		tracker.close();
	}
	
	
	public void getInitialPosition(){
		System.out.println("Getting initial points");
		x=tracker.x;
		y=tracker.y;
		targetX=tracker.targetx;
		targetY=tracker.targety;
		System.out.println("x="+x+"    y="+y);
	}
	
	
	public void calculateJacobian(double a1, double a2){
		System.err.println("Updating Jacobian");
		double[] cor = forwarKin(a1,a2);
		double[] cor_1 = forwarKin(a1+deltaAngle,a2);
		double[] cor_2 = forwarKin(a1,a2+deltaAngle);
		double[][] df = new double[][]
				{
				{cor_1[0]-cor[0],cor_2[0]-cor[0]},
				{cor_1[1]-cor[1],cor_2[1]-cor[1]},
				};
		mdf = new Matrix(df);
		robot.rotateToMotorA((int)a1);
		robot.rotateToMotorB((int)a2);
		System.out.println("["+mdf.get(0, 0)+" "+mdf.get(0, 1));
		System.out.println(mdf.get(1, 0)+" "+mdf.get(1, 1)+"]");
	}
	
	public void inverseNewtonWithInitialGuess(double posX,double posY){
		double[] dA = {0.,0.};
		double difX=(posX-x)/maxGlobalIterations;
		double difY=(posY-y)/maxGlobalIterations;
		
		globalErrors = new double[(int)maxGlobalIterations];
		for (int n=1;n<maxGlobalIterations+1;n++){
			globalErrors[n-1]=Math.hypot(difX*n+x - posX,difY*n+y - posY);
			System.out.println(n+".  "+globalErrors[n-1]);
		}
		
		
		for (int n=1;n<maxGlobalIterations+1;n++){
			System.err.println("ITERACION "+n);
			dA=inverseNewton(difX*n+x,difY*n+y,dA[0],dA[1], n);
			if(n<maxGlobalIterations)
				calculateJacobian(dA[0],dA[1]);
			if(dA[2]>0)
				n=(int)dA[2];
		}
		
		try {
			Thread.sleep(5000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	
	// desired position (pixels), current angles.
	public double[] inverseNewton(double posX, double posY, double a1, double a2, int iteration){
		double[][] dx = new double[][]{{a1},{a2}};
		double[][] y = new double[][]{{posX},{posY}};
		Matrix mdx = new Matrix(dx);
		Matrix my = new Matrix(y);
		double er=-1;
		double error = Double.MAX_VALUE;
		int n=1;
		while(error>minError){
			a1=mdx.get(0, 0);
			a2=mdx.get(1, 0);
			robot.rotateToMotorA((int)a1);
			robot.rotateToMotorB((int)a2);
			
			double[][] f= new double[][]{{tracker.x},{tracker.y}};
			Matrix mf = new Matrix(f);
			
			Matrix minv=mdf.inverse();
			Matrix merr=my.minus(mf);
			Matrix minverr=minv.times(merr);
			Matrix mgain = minverr.times(gain);
			mdx.plusEquals(mgain);
			
			error = Math.hypot(tracker.x-posX, tracker.y-posY);
			System.out.println("Error "+ error);
			n++;
			
			for (int e=iteration;e<maxGlobalIterations;e++){
				double globalError = Math.hypot(tracker.x-targetX, tracker.y-targetY);
				if(globalError<globalErrors[e]){
					System.out.println("Error jump: "+globalError);
					er=e+1;
				}
			}
			if(n>maxLocalIterations)
				break;
			// still has some issues with some areas...
			if(er>0){
				System.err.println("SHORTCUT FOUND! :) , jumping to global iteration:"+(er+1));
				break;
			}
		}
		
		a1 = mdx.get(0, 0);
		a2 = mdx.get(1, 0);
		return new double[]{a1,a2,er};
	}
	
	public double[] forwarKin(double a1, double a2) {
		robot.rotateToMotorA((int)a1);
		robot.rotateToMotorB((int)a2);
		try {Thread.sleep(100);
		} catch (InterruptedException e) {}
		return new double[]{tracker.x,tracker.y};
	}
}
