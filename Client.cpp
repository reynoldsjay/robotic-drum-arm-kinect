#include "Client.h"

void Client::sendAngle1(float x) {
 
	UdpSocket sock;
	sock.connectTo("128.61.70.169", PORT_NUM);
	if (!sock.isOk()) {
		cerr << "Error connection to port " << PORT_NUM << ": " << sock.errorMessage() << "\n";
	}
	else {
		// cout << "Client started, will send packets to port " << PORT_NUM << std::endl;
		int iping = 1;
		Message msg("/drumOneAngle"); 
		msg.pushFloat(x);
		PacketWriter pw;
		pw.startBundle().startBundle().addMessage(msg).endBundle().endBundle();
		bool ok = sock.sendPacket(pw.packetData(), pw.packetSize());
		//cout << "Client: sent /ping " << iping++ << ", ok=" << ok << "\n";
		//cout << "sock error: " << sock.errorMessage() << " -- is the server running?\n";
	}

}

void Client::sendAngle2(float x) {

	UdpSocket sock;
	sock.connectTo("128.61.70.169", PORT_NUM);
	if (!sock.isOk()) {
		cerr << "Error connection to port " << PORT_NUM << ": " << sock.errorMessage() << "\n";
	}
	else {
		// cout << "Client started, will send packets to port " << PORT_NUM << std::endl;
		int iping = 1;
		Message msg("/drumTwoAngle");
		msg.pushFloat(x);
		PacketWriter pw;
		pw.startBundle().startBundle().addMessage(msg).endBundle().endBundle();
		bool ok = sock.sendPacket(pw.packetData(), pw.packetSize());
		//cout << "Client: sent /ping " << iping++ << ", ok=" << ok << "\n";
		//cout << "sock error: " << sock.errorMessage() << " -- is the server running?\n";
	}

}