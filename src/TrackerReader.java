import java.io.DataInputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

public class TrackerReader{
	
    public volatile double x;
    public volatile double y;
    public volatile double basex;
    public volatile double basey;
    public volatile double obstx;
    public volatile double obsty;
    public volatile double a;
    public volatile double theta;
    public volatile double targetx=320;
    public volatile double targety;
    public volatile boolean isConnected = false;
    
    private ServerSocket s = null;
    private Socket conn = null;
    
    //public static void main(String args[]){
    //	TrackerReader tr = new TrackerReader();
    //}
    
    public TrackerReader(){
    	x = y = 0;
    }
    
    public void launchService(){
    	startServer();
    	startClient();
    	while(!isConnected || x==0 || targetx==320){ //connected and with a tracker
    		try {
    			Thread.sleep(10);
    			} catch (InterruptedException e) {
				e.printStackTrace();
			}
    	}
    } 
    
    private void startClient(){
    	Thread client = new Thread(){
    		public void run(){
    			try {
    				Thread.sleep(2000);
    				Runtime.getRuntime().exec("python ./libs/tracker.py");
    				System.out.println("Launched tracker program");
    			} catch (Exception e) {
    				e.printStackTrace();
    			}
    		}
    	};
    	client.start();
    }
    
    private void startServer(){
    	Thread server = new Thread(){
    		@SuppressWarnings("deprecation")
    		public void run(){
    			try{

    				//1. creating a server socket - 1st parameter is port number and 2nd is the backlog
    				s = new ServerSocket(5000 , 10);
    				//2. Wait for an incoming connection
    				conn = s.accept();

    				//print the hostname and port number of the connection
    				System.out.println("Connection received from " + conn.getInetAddress().getHostName() + " : " + conn.getPort());
    				String line = "";
    				JSONParser parser=new JSONParser();
    				try{
    					//get socket writing and reading streams
    					DataInputStream in = new DataInputStream(conn.getInputStream());
    					isConnected = true;
    					//Now start reading input from client
    					while((line = in.readLine()) != null && !line.equals(".")){
    						try{
    							Object obj=parser.parse(line);
    							JSONObject jsonObject = (JSONObject) obj;
    							x = (Double) jsonObject.get("x");
    							y = (Double) jsonObject.get("y");
    							a = (Double) jsonObject.get("a");
    							theta = (Double) jsonObject.get("theta");
    							targetx = (Double) jsonObject.get("targetx");
    							targety = (Double) jsonObject.get("targety");
    						}catch (ParseException e) {
    							e.printStackTrace();
    						}
    						if(!isConnected)
    							break;
    					}
    					//client disconnected, so close socket
    					conn.close();
    				}catch (IOException e){
    					System.out.println("IOException on socket : " + e);
    					e.printStackTrace();
    				}
    			}catch(IOException e){
    				System.err.println("IOException");
    			}

    			//5. close the connections and stream
    			try{
    				s.close();
    			}catch(IOException ioException){
    				System.err.println("Unable to close. IOexception");
    			}
    			isConnected = false;
    		}
    	};
    	server.start();
    }
    public void close(){
    	isConnected=false;
    }
    
}