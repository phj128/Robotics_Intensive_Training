#include "pch.h"
#include<WinSock2.h>
#include<iostream>
using namespace std;
#pragma comment(lib,"ws2_32.lib")

int main(int argc, char* argv[])
{
	WSADATA WSAData;
	WORD sockVersion = MAKEWORD(2, 2);
	if (WSAStartup(sockVersion, &WSAData) != 0)
		return 0;

	SOCKET clientSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (INVALID_SOCKET == clientSocket)
	{
		cout << "socket error!";
		return 0;
	}

	sockaddr_in dstAddr;
	dstAddr.sin_family = AF_INET;
	dstAddr.sin_port = htons(8888);
	dstAddr.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");

	const char* sendData = "来自客户端的数据包。";
	sendto(clientSocket, sendData, strlen(sendData), 0, (sockaddr*)&dstAddr, sizeof(dstAddr));

	closesocket(clientSocket);
	WSACleanup();

	return 0;
}