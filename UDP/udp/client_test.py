#client
import socket
#创建Socket时， SOCK_DGRAM 指定了这个Socket的类型是UDP。
client = socket.socket(type=socket.SOCK_DGRAM)
send_data  =b'hello sheenstar'
client.sendto(send_data,('10.180.35.181',7890))
re_Data,address = client.recvfrom(1024)
print('server>>',re_Data.decode('utf-8'))
client.close()