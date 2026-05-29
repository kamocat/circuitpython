import array
import asyncio
import time

import board

import abusio

spi = abusio.SPI(board.GP2, board.GP3, board.GP4)


async def spier():
    while True:
        while not spi.try_lock():
            await asyncio.sleep(0)
        buf = array.array("h", list(range(1000)))
        tick = time.monotonic()
        for i in range(10):
            await spi.write_readinto(buf, buf)
        # await asyncio.sleep(0)  # Uncomment if you suspect the driver is not yielding
        tock = time.monotonic()
        spi.unlock()
        print(f"Harumph {tock - tick}")


async def talker():
    msg = "Hi there SPI!"
    while True:
        for s in msg.split():
            print(s, end=" ")
            await asyncio.sleep(0)
        print("")
        await asyncio.sleep(1)


async def main():
    await asyncio.gather(spier(), talker())


asyncio.run(main())
