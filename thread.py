import time, threading

x = 1
def print_x():
    for _ in range(100):
        print(x)
        time.sleep(1)

def change_x():
    for _ in range(100):
        global x
        x += 1

t1 = threading.Thread(target=print_x, name='print')
t2 = threading.Thread(target=change_x, name='change')
t1.start()
t2.start()
t1.join()
t2.join()
while True:
    print('thread %s ended.' % threading.current_thread().name)