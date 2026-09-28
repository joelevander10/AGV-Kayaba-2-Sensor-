"""
io_hardware.py — backwards-compatibility shim
==============================================
All driver logic has been extracted to drivers/*.py.
Import from there for new code.
"""
from drivers.modbus_di   import di_reader,   DIReader
from drivers.modbus_do   import do_writer,   DOWriter
from drivers.modbus_ao   import ao_writer,   AOWriter
from drivers.can_mgs1600 import can_reader,  CANReader
from drivers.rfid_tcp    import rfid_reader, RFIDReader

__all__ = [
    "di_reader",   "DIReader",
    "do_writer",   "DOWriter",
    "ao_writer",   "AOWriter",
    "can_reader",  "CANReader",
    "rfid_reader", "RFIDReader",
]
