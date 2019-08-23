import time, threading
from test import test_0, test_5


t1 = threading.Thread(target=test_0, name='blue0')
t2 = threading.Thread(target=test_5, name='blue5')
t1.start()
t2.start()
t1.join()
t2.join()
