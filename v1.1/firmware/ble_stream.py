import ubluetooth
import time
import ujson as json

# Constants
FRAME_START = b'\xff\xd8'
FRAME_END = b'\xff\xd9'
# Reduce chunk size to be safer for standard MTUs and prevent buffer overflow
CHUNK_SIZE = 240  

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
            # Buffer full or disconnected, wait a bit and retry
            time.sleep_ms(10)
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
            # Vital delay to prevent packet loss/buffer overflow
            # 5-10ms is usually required for stable ESP32 BLE streams without flow control
            time.sleep_ms(10)
            
        return True

    def start(self):
        self.streaming = True
        while self.streaming:
            if not self.stream_frame():
                time.sleep_ms(100)
            # Yield briefly between frames
            time.sleep_ms(10)

    def stop(self):
        self.streaming = False
