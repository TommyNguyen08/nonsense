package bluetooth;

public class BTServer {

	public static void main(String[] args) {

		BTConnection conn = new BTConnection();
		conn.start();

        try {
            // Main control loop
        	String message = "";
        	while ((message = conn.getMessage()) != null) {
        		message = message.trim();
        		System.out.println("Msg: " + message);
        	}
            
        } catch (Exception e) {
            System.out.println("Error: " + e.getMessage());
            conn.sendMessage("ERR,parse\n");
            
        } finally {
            conn.close();
            System.out.println("End task");
        }
	}

}
