import os.path as osp
import sys
import toml
import click
from zense_pywrapper import PyPicoZenseManager

import time
import multiprocessing as mp
import psutil

def configure_flipping():
    model = load_model('1.h5')
    x_test = np.random.rand(10000, 1000)
    time.sleep(1)

    for _ in range(3):
        model.predict(x_test)
        time.sleep(0.5)

def monitor(target):
    worker_process = mp.Process(target=target)
    worker_process.start()
    p = psutil.Process(worker_process.pid)

    # log cpu usage of `worker_process` every 10 ms
    cpu_percents = []
    while worker_process.is_alive():
        cpu_percents.append(p.cpu_percent())
        time.sleep(0.01)

    worker_process.join()
    return cpu_percents

cpu_percents = monitor(target=run_predict)