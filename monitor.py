import argparse
import signal
import subprocess
import time

import serial
import trio

parser = argparse.ArgumentParser()
parser.add_argument("-p", "--port", help="Serial port")
args = parser.parse_args()

port = serial.Serial(args.port, 1200)
port.write(b".")
port.flush()
port.close()

time.sleep(1)

subprocess.check_call(
    [
        "/Users/benno/Library/Arduino15/packages/arduino/tools/bossac/1.7.0-arduino3/bossac",
        "-i",
        "-d",
        "-U",
        "true",
        "-e",
        "-w",
        "-v",
        "target/clock2.bin",
        "-R",
        "--port",
        args.port,
    ]
)

time.sleep(3)


async def board_monitor(exit_event):
    async with await trio.open_file(args.port, "r") as port:
        while True:
            print(">>>", exit_event.is_set())
            if exit_event.is_set():
                return
            with trio.move_on_after(1):
                print((await port.readline()).strip())


async def ctrlc_monitor(exit_event):
    with trio.open_signal_receiver(signal.SIGINT) as signal_waiter:
        async for signum in signal_waiter:
            assert signum == signal.SIGINT
            print("YOIKS")
            break
    exit_event.set()


async def monitor():
    exit_event = trio.Event()
    async with trio.open_nursery() as nursery:
        nursery.start_soon(board_monitor, exit_event)
        nursery.start_soon(ctrlc_monitor, exit_event)


trio.run(monitor)
