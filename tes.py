from pymodbus.client import ModbusTcpClient
import threading
import time

print_lock = threading.Lock()

def thread_print(msg):
    with print_lock:
        print(msg)


class CK5162E_Controller:
    def __init__(self, ip_address, port=502):
        self.ip = ip_address
        self.port = port
        self.client = ModbusTcpClient(self.ip, port=self.port)
        self.connected = False
        self.max_channels = 16
        self._input_cache = [False] * 16
        self._last_read_time = 0

    def connect(self):
        try:
            self.connected = self.client.connect()
            return self.connected
        except Exception as e:
            thread_print(f"Error koneksi digital: {e}")
            return False

    def disconnect(self):
        if self.connected:
            self.client.close()
            self.connected = False

    def read_all_inputs(self):
        """Membaca status semua input (discrete input) dari channel 0 hingga max_channels-1.
        Ini sudah 'raw' — langsung query ke device tiap kali dipanggil."""
        if not self.connected:
            return [False] * self.max_channels

        try:
            results = self.client.read_discrete_inputs(0, count=self.max_channels)
            if results.isError():
                thread_print(f"[DIGITAL] read_discrete_inputs error: {results}")
                self.connected = False
                return [False] * self.max_channels
            else:
                self._input_cache = results.bits[:self.max_channels]
                while len(self._input_cache) < self.max_channels:
                    self._input_cache.append(False)
                self._last_read_time = time.time()
                return self._input_cache
        except Exception as e:
            thread_print(f"[DIGITAL] Exception reading inputs: {e}")
            self.connected = False
            return [False] * self.max_channels

    def read_input(self, channel):
        """Membaca input individual - menggunakan cached values dari read_all_inputs"""
        if not self.connected:
            return None
        if channel < 0 or channel >= self.max_channels:
            return None
        if time.time() - self._last_read_time > 0.1:
            self.read_all_inputs()
        return self._input_cache[channel]

    def set_output(self, channel, state):
        if not self.connected:
            return False
        if channel < 0 or channel >= self.max_channels:
            return False
        try:
            result = self.client.write_coil(channel, state)
            return not result.isError()
        except Exception as e:
            thread_print(f"Exception setting output {channel}: {e}")
            self.connected = False
            return False

    def get_output_status(self, channel):
        """Read output coil status (single channel)"""
        if not self.connected:
            return False
        if channel < 0 or channel >= self.max_channels:
            return False
        try:
            result = self.client.read_coils(channel, count=1)
            if result.isError():
                return False
            return result.bits[0] if result.bits else False
        except Exception:
            return False

    def read_all_outputs(self):
        """Membaca status SEMUA output (coil) sekaligus, channel 0 hingga max_channels-1.
        Ini raw read langsung ke device (1x query untuk 16 channel), lebih efisien
        daripada memanggil get_output_status() satu-satu per channel."""
        if not self.connected:
            return [False] * self.max_channels

        try:
            result = self.client.read_coils(0, count=self.max_channels)
            if result.isError():
                thread_print(f"[DIGITAL] read_coils error: {result}")
                self.connected = False
                return [False] * self.max_channels
            else:
                output_status = result.bits[:self.max_channels]
                while len(output_status) < self.max_channels:
                    output_status.append(False)
                return output_status
        except Exception as e:
            thread_print(f"[DIGITAL] Exception reading outputs: {e}")
            self.connected = False
            return [False] * self.max_channels

    def test_connection(self):
        """Test connection to controller"""
        try:
            if not self.connected:
                return False
            result = self.client.read_coils(0, count=1)
            return not result.isError()
        except Exception:
            return False


def print_raw_io(ctrl: CK5162E_Controller):
    """Cetak raw semua input & output digital dalam satu baris yang gampang dibaca."""
    inputs = ctrl.read_all_inputs()
    outputs = ctrl.read_all_outputs()

    in_str = "".join("1" if b else "0" for b in inputs)
    out_str = "".join("1" if b else "0" for b in outputs)

    thread_print(f"INPUT  (ch0->ch15): {in_str}  raw={inputs}")
    thread_print(f"OUTPUT (ch0->ch15): {out_str}  raw={outputs}")


if __name__ == "__main__":
    DIGITAL_IO_IP = "192.168.1.30"   # <-- ganti dengan IP CK5162E kamu
    DIGITAL_IO_PORT = 502

    digital_io = CK5162E_Controller(DIGITAL_IO_IP, DIGITAL_IO_PORT)

    if not digital_io.connect():
        thread_print("Gagal konek ke digital I/O controller.")
        raise SystemExit(1)

    thread_print("Terkoneksi ke digital I/O. Ctrl+C untuk berhenti.\n")

    try:
        while True:
            print_raw_io(digital_io)
            digital_io.set_output(0, True)
            digital_io.set_output(1, True)
            digital_io.set_output(6, True)
            digital_io.set_output(7, True)
            thread_print("-" * 60)
            time.sleep(1)
            digital_io.set_output(0, False)
            digital_io.set_output(1, False)
            digital_io.set_output(6, False)
            digital_io.set_output(7, False)
            time.sleep(1)
    except KeyboardInterrupt:
        thread_print("Dihentikan oleh user.")
    finally:
        digital_io.disconnect()