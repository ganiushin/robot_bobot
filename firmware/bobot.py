import bluetooth
import ubluetooth
import ujson as json
import ubinascii
import time
import sys, io, os, builtins

try:
    import _thread
except Exception:
    _thread = None

try:
    import ble_stream
except ImportError:
    ble_stream = None

_CAMERA_AVAILABLE = False
camera = None
cam_instance = None
camera_initialized = False
_http_thread_running = False
_http_stop_flag = None
try:
    import camera
    _CAMERA_AVAILABLE = True
except Exception:
    camera = None
    _CAMERA_AVAILABLE = False

_UART_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX   = (bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"), ubluetooth.FLAG_NOTIFY)
_UART_RX   = (bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"), ubluetooth.FLAG_WRITE | ubluetooth.FLAG_WRITE_NO_RESPONSE)
_STREAM_TX = (bluetooth.UUID("6E400004-B5A3-F393-E0A9-E50E24DCCA9E"), ubluetooth.FLAG_NOTIFY)
_UART_SVC  = (_UART_UUID, (_UART_TX, _UART_RX, _STREAM_TX))
_ADV_NAME  = b"MPY-ESP32-BLE"

class BLEUART:
    def __init__(self, on_rx):
        self._ble = bluetooth.BLE()
        self._ble.active(True)
        self._ble.irq(self._irq)
        ((self._tx_handle, self._rx_handle, self._stream_handle),) = self._ble.gatts_register_services((_UART_SVC,))
        self._connections = set()
        self._on_rx = on_rx
        self._ble.gatts_set_buffer(self._rx_handle, 512, True)
        self._advertise()

    def _irq(self, event, data):
        if event == 1:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
        elif event == 2:
            conn_handle, _, _ = data
            self._connections.discard(conn_handle)
            self._advertise()
        elif event == 3:
            conn_handle, attr_handle = data
            if attr_handle == self._rx_handle and self._on_rx:
                try:
                    buf = self._ble.gatts_read(self._rx_handle)
                    self._on_rx(buf)
                except Exception as e:
                    self.notify(json.dumps({"op":"err","s":repr(e)}) + "\n")

    def notify(self, data):
        if not isinstance(data, bytes):
            data = data.encode()
        for conn in self._connections:
            try:
                self._ble.gatts_notify(conn, self._tx_handle, data)
            except:
                pass
    
    def get_first_connection(self):
        if self._connections:
            return list(self._connections)[0]
        return None

    def _advertise(self):
        name = _ADV_NAME
        adv = bytearray(b"\x02\x01\x06") + bytearray((len(name)+1, 0x09)) + name
        uuid_le = b"\x9e\xca\xdc\x24\x0e\xe5\xa9\xe0\x93\xf3\xa3\xb5\x01\x00\x40\x6e"
        sr = bytearray((len(uuid_le)+1, 0x07)) + uuid_le
        self._ble.gap_advertise(100_000, adv_data=adv, resp_data=sr)

class CaptureIO:
    def __init__(self):
        self.buf = io.StringIO()
    def write(self, s):
        self.buf.write(s)
    def getvalue(self):
        return self.buf.getvalue()

class PutSession:
    def __init__(self, path, size):
        self.path = path
        self.size = size
        self.received = 0
        self.f = open(path, "wb")
    def write_chunk_b64(self, b64s):
        data = ubinascii.a2b_base64(b64s)
        self.f.write(data)
        self.received += len(data)
    def close(self):
        try:
            self.f.close()
        except:
            pass

put_session = None
uart = None
rx_buffer = b""
CAMERA_CHUNK = 600

# Global streaming state
stream_active = False
stream_handler = None
ai_result = None

def wait_for_ai():
    global ai_result
    ai_result = None
    if not uart: return None
    uart.notify('{"op":"ai_predict"}\n')
    t = time.ticks_ms()
    while ai_result is None:
        time.sleep_ms(10)
        if time.ticks_diff(time.ticks_ms(), t) > 2000:
            return "timeout" # Timeout
    return ai_result

def camera_deinit():
    global cam_instance
    if not _CAMERA_AVAILABLE:
        return
    try:
        if cam_instance and hasattr(cam_instance, 'deinit'):
            cam_instance.deinit()
        elif hasattr(camera, 'deinit'):
            camera.deinit()
    except Exception:
        pass
    cam_instance = None


def camera_init(config):
    global cam_instance
    if not _CAMERA_AVAILABLE:
        return False, "camera module not available"
    
    try:
        from camera import Camera, FrameSize, PixelFormat

        # Defaults
        framesize_str = str((config or {}).get('framesize', 'HQVGA')).upper()
        framesize_val = getattr(FrameSize, framesize_str, FrameSize.HQVGA)
        
        if not cam_instance:
            # Initialize using known working hardware config (ESP32-S3-CAR)
            cam_instance = Camera(
                frame_size=framesize_val,
                pixel_format=PixelFormat.JPEG,
                init=False,
                data_pins=[19, 17, 16, 18, 20, 48, 47, 42],
                vsync_pin=46,
                href_pin=45,
                sda_pin=1,
                scl_pin=2,
                pclk_pin=21,
                xclk_pin=41,
                xclk_freq=10000000, # 10 MHz
                powerdown_pin=5,
                reset_pin=-1,
            )
            cam_instance.init()
        else:
            # Reconfigure if possible
            try:
                if hasattr(cam_instance, 'reconfigure'):
                    cam_instance.reconfigure(frame_size=framesize_val)
            except:
                pass
        
        # Optional quality setting
        try:
            if hasattr(cam_instance, 'set_quality'):
                cam_instance.set_quality(85)
        except Exception:
            pass
            
        time.sleep(0.5)
        return True, None

    except Exception as exc:
        return False, repr(exc)


def camera_capture(config):
    global cam_instance
    if not _CAMERA_AVAILABLE:
        return False, "camera module not available", None
    
    if not cam_instance:
         ok, err = camera_init(config)
         if not ok:
             return False, "camera not initialized: " + err, None

    try:
        buf = None
        try:
            buf = cam_instance.capture()
        except Exception:
            pass
        
        if not buf:
            time.sleep_ms(50)
            try:
                 buf = cam_instance.capture()
            except:
                 pass
                
        if not buf:
            return False, "empty buffer", None
            
        try:
            if hasattr(buf, 'tobytes'):
                buf = buf.tobytes()
            elif isinstance(buf, memoryview):
                buf = buf.tobytes()
            elif not isinstance(buf, (bytes, bytearray)):
                buf = bytes(buf)
        except Exception:
            return False, "buffer conversion failed", None
            
        b64 = ubinascii.b2a_base64(buf).decode().replace('\n','')
        return True, None, b64
    except Exception as exc:
        return False, repr(exc), None


def _ensure_camera_ready_for_http(framesize='QVGA', quality=85):
    global cam_instance
    ok, err = camera_init({'framesize': framesize})
    if not ok:
        return False, err
    try:
        if cam_instance and hasattr(cam_instance, 'set_quality'):
            cam_instance.set_quality(quality)
    except Exception:
        pass
    return True, None

def list_dir(path):
    try:
        entries = []
        for it in os.ilistdir(path):
            if isinstance(it, tuple):
                entries.append(it[0])
            else:
                entries.append(it)
        return {"ok":True, "entries":entries}
    except Exception as e:
        return {"ok":False, "err":repr(e)}

def handle_json(cmd):
    global put_session, stream_active
    op = cmd.get("op")

    if op == "put":
        path = cmd.get("path", "/uploaded.bin")
        size = int(cmd.get("size", 0))
        try:
            parts = path.split("/")
            if len(parts) > 2:
                d = "/"
                for p in parts[1:-1]:
                    if not p:
                        continue
                    d = d + ("" if d.endswith("/") else "/") + p
                    try:
                        os.mkdir(d)
                    except OSError:
                        pass
            put_session = PutSession(path, size)
            uart.notify(json.dumps({"op":"ack","phase":"put","ok":True}) + "\n")
        except Exception as e:
            uart.notify(json.dumps({"op":"ack","phase":"put","ok":False,"err":repr(e)}) + "\n")

    elif op == "data":
        if not put_session:
            uart.notify(json.dumps({"op":"ack","phase":"data","ok":False,"err":"no put session"}) + "\n")
            return
        try:
            b64s = cmd.get("b64", "")
            put_session.write_chunk_b64(b64s)
            uart.notify(json.dumps({"op":"ack","phase":"data","ok":True,"received":put_session.received}) + "\n")
        except Exception as e:
            uart.notify(json.dumps({"op":"ack","phase":"data","ok":False,"err":repr(e)}) + "\n")

    elif op == "end":
        if not put_session:
            uart.notify(json.dumps({"op":"ack","phase":"end","ok":False,"err":"no put session"}) + "\n")
            return
        try:
            ok = (put_session.received == put_session.size) if put_session.size else True
            path = put_session.path
            put_session.close()
            put_session = None
            uart.notify(json.dumps({"op":"ack","phase":"end","ok":ok,"path":path}) + "\n")
        except Exception as e:
            uart.notify(json.dumps({"op":"ack","phase":"end","ok":False,"err":repr(e)}) + "\n")

    elif op == "run":
        code_b64 = cmd.get("code_b64", "")
        uart.notify(json.dumps({"op":"ack","phase":"run","stage":"received"}) + "\n")
        
        def run_user_code():
            code = ubinascii.a2b_base64(code_b64).decode()
            cap_out = CaptureIO()
            cap_err = CaptureIO()
            # We need to capture print per thread? builtins.print is global.
            # This is tricky in threading. 
            # Simplified: just run and hope for best or use thread-local storage if available (not in MP).
            # For now, just redirect global print. If main loop prints, it gets captured too. Acceptable.
            old_print = builtins.print
            def patched_print(*args, **kwargs):
                sep = kwargs.get("sep", " ")
                end = kwargs.get("end", "\n")
                msg = sep.join(str(a) for a in args) + end
                cap_out.write(msg)
                # Also print to real stdout for debug
                # old_print(msg, end='') 
            
            builtins.print = patched_print
            try:
                try:
                    # Execute
                    exec(code, globals(), globals())
                except Exception as e:
                    if hasattr(sys, "print_exception"):
                        sys.print_exception(e, cap_err)
                    else:
                        cap_err.write("ERR: " + repr(e) + "\n")
            finally:
                builtins.print = old_print
                
            if cap_out.getvalue():
                uart.notify(json.dumps({"op":"out","s":cap_out.getvalue()}) + "\n")
            if cap_err.getvalue():
                uart.notify(json.dumps({"op":"err","s":cap_err.getvalue()}) + "\n")
            uart.notify(json.dumps({"op":"ack","phase":"run","ok":True}) + "\n")

        if _thread:
            _thread.start_new_thread(run_user_code, ())
        else:
            # Fallback to blocking
            run_user_code()

    elif op == "ls":
        path = cmd.get("path","/")
        uart.notify(json.dumps({"op":"ls","res":list_dir(path)}) + "\n")

    elif op == "rm":
        path = cmd.get("path","")
        try:
            os.remove(path)
            uart.notify(json.dumps({"op":"ack","phase":"rm","ok":True,"path":path}) + "\n")
        except Exception as e:
            uart.notify(json.dumps({"op":"ack","phase":"rm","ok":False,"err":repr(e)}) + "\n")

    elif op == "camera_init":
        config = cmd.get("config", {})
        uart.notify(json.dumps({"op":"ack","phase":"camera_init","stage":"received"}) + "\n")
        ok, err = camera_init(config)
        uart.notify(json.dumps({"op":"ack","phase":"camera_init","ok":ok,"err":err if not ok else None}) + "\n")

    elif op == "camera_deinit":
        camera_deinit()
        uart.notify(json.dumps({"op":"ack","phase":"camera_deinit","ok":True}) + "\n")

    elif op == "camera_capture":
        config = cmd.get("config", {})
        try:
            uart.notify(json.dumps({"op":"ack","phase":"camera_capture","stage":"received"}) + "\n")
        except Exception:
            pass
        try:
            ok, err, data_b64 = camera_capture(config)
            if not ok:
                uart.notify(json.dumps({"op":"ack","phase":"camera_capture","ok":False,"err":err}) + "\n")
                return
            total = len(data_b64)
            index = 0
            for offset in range(0, total, CAMERA_CHUNK):
                chunk = data_b64[offset:offset+CAMERA_CHUNK]
                uart.notify(json.dumps({"op":"camera_chunk","index":index,"final":False,"data":chunk}) + "\n")
                index += 1
            uart.notify(json.dumps({"op":"camera_chunk","index":index,"final":True,"data":"","size":total}) + "\n")
            uart.notify(json.dumps({"op":"ack","phase":"camera_capture","ok":True,"chunks":index+1,"size":total}) + "\n")
        except Exception as e:
            uart.notify(json.dumps({"op":"ack","phase":"camera_capture","ok":False,"err":repr(e)}) + "\n")

    # Stream Ops
    elif op == "stream_start":
        if not ble_stream or not uart:
             uart.notify(json.dumps({"op":"ack","phase":"stream_start","ok":False,"err":"not supported"}) + "\n")
             return
        
        config = cmd.get("config", {})
        
        if not cam_instance:
            ok, err = camera_init(config)
            if not ok:
                 uart.notify(json.dumps({"op":"ack","phase":"stream_start","ok":False,"err":err}) + "\n")
                 return
        
        conn = uart.get_first_connection()
        if conn:
            global stream_handler
            # Pass the separate stream handle
            stream_handler = ble_stream.BLEStream(uart._ble, conn, uart._stream_handle, cam_instance)
            stream_active = True
            uart.notify(json.dumps({"op":"ack","phase":"stream_start","ok":True}) + "\n")
        else:
             uart.notify(json.dumps({"op":"ack","phase":"stream_start","ok":False,"err":"no connection"}) + "\n")

    elif op == "stream_stop":
        stream_active = False
        if stream_handler:
            stream_handler.streaming = False
        uart.notify(json.dumps({"op":"ack","phase":"stream_stop","ok":True}) + "\n")

    elif op == "ai_result":
        global ai_result
        ai_result = cmd.get("class")

    # Start AP WiFi
    elif op == "wifi_ap_start":
        cfg = cmd.get('config', {})
        ssid = cfg.get('ssid', 'ESP32-CAM')
        password = cfg.get('password', '')
        channel = cfg.get('channel', 6)
        try:
            try:
                import cam_wifi
            except Exception as e:
                uart.notify(json.dumps({"op":"ack","phase":"wifi_ap_start","ok":False,"err":"import cam_wifi failed: " + repr(e)}) + "\n")
                return
            ifcfg = cam_wifi.start_ap(ssid=ssid, password=password, channel=channel)
            uart.notify(json.dumps({"op":"ack","phase":"wifi_ap_start","ok":True,"ifconfig":ifcfg}) + "\n")
        except Exception as e:
            uart.notify(json.dumps({"op":"ack","phase":"wifi_ap_start","ok":False,"err":repr(e)}) + "\n")

    elif op == "wifi_ap_stop":
        try:
            import cam_wifi
            cam_wifi.stop_ap()
            uart.notify(json.dumps({"op":"ack","phase":"wifi_ap_stop","ok":True}) + "\n")
        except Exception as e:
            uart.notify(json.dumps({"op":"ack","phase":"wifi_ap_stop","ok":False,"err":repr(e)}) + "\n")

    # Start/Stop HTTP camera server
    elif op == "http_cam_start":
        cfg = cmd.get('config', {})
        framesize = str(cfg.get('framesize', 'QVGA')).upper()
        quality = int(cfg.get('quality', 85))
        try:
            uart.notify(json.dumps({"op":"ack","phase":"http_cam_start","stage":"received"}) + "\n")
        except Exception:
            pass
        try:
            ok, err = _ensure_camera_ready_for_http(framesize, quality)
            if not ok:
                uart.notify(json.dumps({"op":"ack","phase":"http_cam_start","ok":False,"err":err}) + "\n")
                return
            try:
                import cam_http
                if cam_instance:
                    cam_http.set_camera(cam_instance)
            except Exception as e:
                uart.notify(json.dumps({"op":"ack","phase":"http_cam_start","ok":False,"err":"import cam_http failed: " + repr(e)}) + "\n")
                return
            global _http_thread_running, _http_stop_flag
            if _http_thread_running:
                uart.notify(json.dumps({"op":"ack","phase":"http_cam_start","ok":True,"note":"already running"}) + "\n")
                return
            _http_stop_flag = {'stop': False}
            if _thread is None:
                uart.notify(json.dumps({"op":"ack","phase":"http_cam_start","ok":False,"err":"_thread not available"}) + "\n")
                return
            def _runner():
                try:
                    cam_http.serve(_http_stop_flag)
                finally:
                    global _http_thread_running
                    _http_thread_running = False
            _http_thread_running = True
            _thread.start_new_thread(_runner, ())
            uart.notify(json.dumps({"op":"ack","phase":"http_cam_start","ok":True}) + "\n")
        except Exception as e:
            uart.notify(json.dumps({"op":"ack","phase":"http_cam_start","ok":False,"err":repr(e)}) + "\n")

    elif op == "http_cam_stop":
        try:
            global _http_stop_flag
            if _http_stop_flag is not None:
                _http_stop_flag['stop'] = True
            uart.notify(json.dumps({"op":"ack","phase":"http_cam_stop","ok":True}) + "\n")
        except Exception as e:
            uart.notify(json.dumps({"op":"ack","phase":"http_cam_stop","ok":False,"err":repr(e)}) + "\n")


def on_rx(buf):
    global rx_buffer
    rx_buffer += buf
    while True:
        idx = rx_buffer.find(b"\n")
        if idx < 0:
            break
        line = rx_buffer[:idx].strip()
        rx_buffer = rx_buffer[idx+1:]
        if not line:
            continue
        try:
            cmd = json.loads(line)
            handle_json(cmd)
        except Exception as e:
            uart.notify(json.dumps({"op":"err","s":"bad json: " + repr(e)}) + "\n")

def main():
    global uart
    uart = BLEUART(on_rx)
    
    while True:
        if stream_active and stream_handler:
             # Send one frame, then yield to check for RX
             stream_handler.stream_frame()
             time.sleep_ms(10) # Yield to stack
        else:
             time.sleep_ms(100)
             
        # Optional heartbeat if not streaming
        if not stream_active and time.time() % 5 == 0:
             try:
                 uart.notify(b'{"op":"ping"}\n')
             except:
                 pass

if __name__ == "__main__":
    main()
