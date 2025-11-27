import ubluetooth
import time
import ujson as json

# Constants
FRAME_START = b'\xff\xd8'
FRAME_END = b'\xff\xd9'
CHUNK_SIZE = 128  # Increased from 20 for better throughput
# Note: Client must support this MTU or handle reassembly. 
# Standard BLE MTU is 23 (20 payload). Requesting larger MTU is client side.

class BLEStream:
    def __init__(self, ble, conn_handle, tx_handle, camera_instance):
        self.ble = ble
        self.conn_handle = conn_handle
        self.tx_handle = tx_handle
        self.cam = camera_instance
        self.streaming = True
        
    def send_data(self, data):
        if not self.conn_handle: return
        try:
            self.ble.gatts_notify(self.conn_handle, self.tx_handle, data)
        except OSError:
            # Buffer full or disconnected, wait a bit
            time.sleep_ms(5)
            try:
                self.ble.gatts_notify(self.conn_handle, self.tx_handle, data)
            except:
                pass

    def stream_frame(self):
        if not self.cam: return False
        
        try:
            frame = self.cam.capture()
        except:
            return False
            
        if not frame: return False
        
        length = len(frame)
        offset = 0
        
        # Send frame in chunks
        while offset < length and self.streaming:
            chunk = frame[offset:offset+CHUNK_SIZE]
            self.send_data(chunk)
            offset += CHUNK_SIZE
            # Minimal delay to allow stack processing but maximize speed
            # time.sleep_ms(1) 
            
        return True

    def start(self):
        self.streaming = True
        while self.streaming:
            if not self.stream_frame():
                time.sleep_ms(100)
            # Yield briefly
            time.sleep_ms(10)

    def stop(self):
        self.streaming = False
