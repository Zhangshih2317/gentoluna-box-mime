import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.EOFException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;

/** Receives framed robot feedback over ADB and delivers it to Apex on loopback. */
public final class PicoFeedbackRelay {
    public static void main(String[] args) throws Exception {
        InetAddress loopback = InetAddress.getByName("127.0.0.1");
        try (ServerSocket server = new ServerSocket(39002, 1, loopback)) {
            server.setSoTimeout(30000);
            System.out.println("LISTENING 127.0.0.1:39002");
            System.out.flush();
            try (Socket client = server.accept();
                 DatagramSocket udp = new DatagramSocket(0, loopback);
                 DataInputStream input = new DataInputStream(client.getInputStream());
                 DataOutputStream output = new DataOutputStream(client.getOutputStream())) {
                client.setTcpNoDelay(true);
                client.setSoTimeout(15000);
                long left = 0;
                long right = 0;
                long lastReport = 0;
                byte[] buffer = new byte[8192];
                while (true) {
                    int port;
                    try {
                        port = input.readUnsignedShort();
                    } catch (EOFException closed) {
                        break;
                    }
                    int length = input.readUnsignedShort();
                    if (port == 0 && length == 0) {
                        output.writeBytes("{\"delivered9002\":" + left + ",\"delivered9003\":" + right + "}\n");
                        output.flush();
                        continue;
                    }
                    if ((port != 9002 && port != 9003) || length < 1 || length > buffer.length) {
                        throw new IllegalArgumentException("Invalid feedback frame");
                    }
                    input.readFully(buffer, 0, length);
                    udp.send(new DatagramPacket(buffer, length, loopback, port));
                    if (port == 9002) left++; else right++;
                    long now = System.currentTimeMillis();
                    if (now - lastReport >= 5000) {
                        String status = "{\"delivered9002\":" + left + ",\"delivered9003\":" + right + "}\n";
                        output.writeBytes(status);
                        output.flush();
                        System.out.print(status);
                        System.out.flush();
                        lastReport = now;
                    }
                }
            }
        }
    }
}
