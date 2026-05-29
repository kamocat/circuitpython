"""
busio SPI verification script — use this to rule out hardware/wiring issues
before debugging the abusio async driver.

Pinout (matches logic analyzer wiring in .vscode/hardware-setup.md):
  GP1 — CS   (D6 on fx2lafw)
  GP2 — SCK  (D5 on fx2lafw)
  GP3 — MOSI (D1 on fx2lafw)
  GP4 — MISO (D3 on fx2lafw)

Expected on the logic analyzer: MOSI counts 0x00..0xFF repeatedly.
If MOSI is all 0xFF here too, the problem is hardware/wiring, not the
async driver.
"""

import array

import board
import busio
import digitalio

COUNT = 256  # one full 0x00-0xFF sweep — easy to spot on the analyzer

spi = busio.SPI(board.GP2, board.GP3, board.GP4)  # SCK, MOSI, MISO

cs = digitalio.DigitalInOut(board.GP1)
cs.direction = digitalio.Direction.OUTPUT
cs.value = True

while not spi.try_lock():
    pass
spi.configure(baudrate=100_000, polarity=0, phase=0)

tx = array.array("B", range(COUNT))  # 0x00, 0x01, … 0xFF
rx = array.array("B", [0] * COUNT)

import time

print("Looping 0x00-0xFF over SPI (busio) — arm the logic analyzer now…")
while True:
    cs.value = False
    spi.write_readinto(tx, rx)
    cs.value = True
    time.sleep(0.1)
