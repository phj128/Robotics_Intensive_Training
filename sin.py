import numpy as np
import math
import time
import random

start = time.time()
for _ in range(1000000):
    x = np.random.randint(-300, 300)
end = time.time()
print('np:', end-start)


start = time.time()
for _ in range(1000000):
    x = random.randint(-300, 300)
end = time.time()
print('math:', end-start)

start = time.time()
for _ in range(10000):
    x = 100 * 100
end = time.time()
print('alone:', end-start)