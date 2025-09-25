import asyncio
from pymodbus.server.async_io import StartAsyncSerialServer, StartSerialServer
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext

import logging
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.DEBUG)

# Buat data block (10 register)
store = ModbusSlaveContext(
    di=ModbusSequentialDataBlock(0, [0]*100),  # Discrete Inputs
    co=ModbusSequentialDataBlock(0, [0]*100),  # Coils
    hr=ModbusSequentialDataBlock(0, [10,20,30,40,50,60,70,80,90,100]),  # Holding Registers
    ir=ModbusSequentialDataBlock(0, [0]*100),  # Input Registers
)

context = ModbusServerContext(slaves={0x01: store}, single=False)

async def run_server():
    await StartAsyncSerialServer(
        context=context,
        port="/dev/ttyS0",
        baudrate=57600,
        parity="N",
        stopbits=1,
        bytesize=8,
        framer=None,  # default RTU
    )

if __name__ == "__main__":
    # asyncio.run(run_server())
    StartSerialServer(
        context=context,
        port="/dev/ttyS0",
        baudrate=57600,
        parity="N",
        stopbits=1,
        bytesize=8,
        framer=None,  # default RTU
    )
