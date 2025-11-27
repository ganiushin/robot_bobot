import socket
import time

try:
    import camera
except ImportError:
    camera = None

_camera_instance = None

def set_camera(cam):
    global _camera_instance
    _camera_instance = cam

try:
    from cam_html import INDEX
except ImportError:
    INDEX = b"HTTP/1.1 200 OK\r\n\r\nOK"


BOUNDARY = b"frame"


def _jpg_snapshot():
    cam = _camera_instance if _camera_instance else camera
    if not cam:
        return None
    buf = cam.capture()
    if not buf:
        return None
    try:
        if hasattr(buf, 'tobytes'):
            buf = buf.tobytes()
        elif isinstance(buf, memoryview):
            buf = buf.tobytes()
        elif not isinstance(buf, (bytes, bytearray)):
            buf = bytes(buf)
    except Exception:
        pass
    return buf


def serve(stop_flag, addr=('0.0.0.0', 80)):
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(addr)
    s.listen(2)
    try:
        while not stop_flag.get('stop'):
            try:
                s.settimeout(0.5)
                cl, _ = s.accept()
            except OSError:
                continue
            try:
                req = cl.recv(512)
                if not req:
                    cl.close()
                    continue
                line = req.split(b"\r\n", 1)[0]
                parts = line.split()
                path = parts[1] if len(parts) >= 2 else b"/"
                if path.startswith(b"/webcam"):
                    hdr = (
                        b"HTTP/1.1 200 OK\r\n"
                        b"Content-Type: multipart/x-mixed-replace; boundary=" + BOUNDARY + b"\r\n"
                        b"Cache-Control: no-cache\r\n\r\n"
                    )
                    cl.send(hdr)
                    for _ in range(2000):
                        if stop_flag.get('stop'):
                            break
                        img = _jpg_snapshot()
                        if not img:
                            time.sleep_ms(50)
                            continue
                        cl.send(b"--" + BOUNDARY + b"\r\n")
                        cl.send(b"Content-Type: image/jpeg\r\n")
                        cl.send(b"Content-Length: %d\r\n\r\n" % len(img))
                        cl.send(img)
                        cl.send(b"\r\n")
                        time.sleep_ms(80)
                    cl.close()
                elif path.startswith(b"/snap"):
                    img = _jpg_snapshot()
                    if not img:
                        cl.send(b"HTTP/1.1 503 Service Unavailable\r\n\r\nno image")
                    else:
                        cl.send(b"HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nCache-Control: no-cache\r\n\r\n")
                        cl.send(img)
                    cl.close()
                else:
                    if isinstance(INDEX, str):
                        cl.send(INDEX)
                    else:
                        cl.send(INDEX)
                    cl.close()
            except Exception:
                try:
                    cl.close()
                except Exception:
                    pass
    finally:
        try:
            s.close()
        except Exception:
            pass


