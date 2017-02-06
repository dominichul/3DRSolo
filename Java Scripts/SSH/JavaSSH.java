import com.jcraft.jsch.*;
import java.io.*;
import java.util.*;

public class JavaSSH {

	public static void main(String[] arg) {

		// Server config
		String remoteHostName = "127.0.0.1";
		String remoteHostUserName = "crazymao";
		String remoteHostpassword = "password";

		String scriptLaunchCommand = "python script.py";

		try {
			// New jsch instance
			JSch jsch = new JSch();

			// Set known ssh hosts, otherwise it complains
			jsch.setKnownHosts("~/.ssh/known_hosts");

			// Create ssh session using server config above and port 22
			Session session = jsch.getSession(remoteHostUserName, remoteHostName, 22);
			session.setPassword(remoteHostpassword);

			// Disable prompts and add to session settings
			Properties config = new Properties();
			config.put("StrictHostKeyChecking", "no");
			session.setConfig(config);

			// Connect session
			session.connect();

			// Create and send command to execute remotely
			ChannelExec channel = (ChannelExec) session.openChannel("exec");
			BufferedReader in = new BufferedReader(new InputStreamReader(channel.getInputStream()));
			channel.setCommand(scriptLaunchCommand);
			channel.connect();

			// Get date for file name
			Calendar now = Calendar.getInstance();
			int year = now.get(Calendar.YEAR);
			int month = now.get(Calendar.MONTH) + 1; // Note: zero based!
			int day = now.get(Calendar.DAY_OF_MONTH);
			int hour = now.get(Calendar.HOUR_OF_DAY);
			int minute = now.get(Calendar.MINUTE);
			int second = now.get(Calendar.SECOND);

			String dataReceived = null; // Data received from script/server to be parsed
			BufferedWriter bw = null;
			FileWriter fw = null;

			String filename = year + "-" + month + "-" + day + " " + hour + "-" + minute + "-" + second + ".txt"; // File to write to locally not on the server!!
			File file = new File(filename); // File instance
			// If file doesnt exists, then create it
			if (!file.exists()) {
				file.createNewFile();
			}

			// Parse data being sent back
			while ((dataReceived = in.readLine()) != null) {
				try {
					fw = new FileWriter(file.getAbsoluteFile(), true); // True - append to file
					bw = new BufferedWriter(fw);

					bw.write(dataReceived);
					bw.write("\n");

				} catch (IOException e) {
					e.printStackTrace();
				} finally {
					// Close everything
					try {
						if (bw != null)
							bw.close();

						if (fw != null)
							fw.close();
					} catch (IOException ex) {
						ex.printStackTrace();
					}
				}
			}

			// Disconnect from everything
			channel.disconnect();
			session.disconnect();
		} catch (Exception ex) {
			ex.printStackTrace();
		}
	}
}
