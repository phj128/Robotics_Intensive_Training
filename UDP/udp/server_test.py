#server
import socket

#创建Socket时， SOCK_DGRAM 指定了这个Socket的类型是UDP。
server = socket.socket(type=socket.SOCK_DGRAM)
server.bind(('10.180.35.181',7890))

print('服务端已开启7890端口，正在等待被连接...')
#recvfrom() 方法返回数据和客户端的地址与端口， 这样， 服务器收到数据后，
#直接调用 sendto() 就可以把数据用UDP发给客户端
data,address = server.recvfrom(1024)
print("client>>",data.decode('utf-8'))
print("客户端连接的socket地址：",  address)
server.sendto(b'drink more water!',address)
server.close()