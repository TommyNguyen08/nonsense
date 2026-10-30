package bluetooth;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import lejos.remote.nxt.BTConnector;
import lejos.remote.nxt.NXTConnection;

public class BTConnection {
	// Connection
    private static DataInputStream input;
    private static DataOutputStream output;
    private static BTConnector connector;
    private static NXTConnection connection;
    
    public BTConnection() {
    	connector = new BTConnector();
    }
    
    public void start() {
    	// Wait for Bluetooth connection
    	connection = connector
        		.waitForConnection(0, NXTConnection.RAW);
    	if (connection == null) {
            System.out.println("Connection failed");
            return;
        }
        System.out.println("Connected!");

        input = connection.openDataInputStream();
        output = connection.openDataOutputStream();
    }
    
    public void close() {
    	try {
            if (input != null) input.close();
            if (output != null) output.close();
            if (connection != null) connection.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    
    public String getMessage() {
    	String msg = null;
		try {
			msg = input.readLine();
		} catch (IOException e) {
			e.printStackTrace();
		}
    	return msg;
    }
    
    public void sendMessage(String str)  {
    	try {
			output.writeBytes(str);
		} catch (IOException e) {
			e.printStackTrace();
		}
    }
}
