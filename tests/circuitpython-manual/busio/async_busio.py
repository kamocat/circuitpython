import asyncio
import busio
import board
import time
import array
import time

addr = 31

async def i2c_lock(i2c):
    while not i2c.try_lock():
        await asyncio.sleep(0)

async def i2c_write(i2c, buf):
    d = busio.dma.i2c_write(i2c, addr, buf)
    while busio.dma.i2c_is_busy(d):
        await asyncio.sleep(0)

async def i2c_read(i2c, buf):
    d = busio.dma.i2c_read(i2c, addr, buf)
    while busio.dma.i2c_is_busy(d):
        await asyncio.sleep(0)

async def icer():
    i2c = busio.I2C(board.GP21, board.GP20)

    
    register = bytearray([0x00])
    data = bytearray(1)
    while True:
        await i2c_lock(i2c)
        bin = array.array('h', [1]*100)
        bout = array.array('b', [0])
        tick = time.monotonic()
        await i2c_write(i2c, bout)
        await i2c_read(i2c, bin )
        #Uncomment this asyncio.sleep and see the difference
        #await asyncio.sleep(0)
        tock = time.monotonic()
        i2c.unlock()
        print(f'Hush! I am very busy! {tock-tick} ')
        time.sleep(1) #being very busy

async def talker():
    msg = 'But what about MEEE ??!'
    while True:
        for s in msg.split():
            print(s)
            await asyncio.sleep(0)
        await asyncio.sleep(1)

async def main():
    await asyncio.gather(icer(), talker())

asyncio.run(main())
