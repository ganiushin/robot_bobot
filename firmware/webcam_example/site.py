import socket
import time
import ubinascii

try:
    import camera
except ImportError:
    camera = None

try:
    from html import INDEX
except ImportError:
    from .html import INDEX


BOUNDARY = "frame"


def _jpg_snapshot():
    if not camera:
        return None
    buf = camera.capture()
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


def serve(addr=('0.0.0.0', 80)):
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(addr)
    s.listen(2)
    while True:
        cl, _ = s.accept()
        try:
            req = cl.recv(512)
            if not req:
                cl.close()
                continue
            line = req.split(b"\r\n", 1)[0]
            path = line.split()[1] if len(line.split()) >= 2 else b"/"
            if path.startswith(b"/webcam"):
                hdr = (
                    b"HTTP/1.1 200 OK\r\n"
                    b"Content-Type: multipart/x-mixed-replace; boundary=%s\r\n"
                    b"Cache-Control: no-cache\r\n\r\n" % BOUNDARY.encode()
                )
                cl.send(hdr)
                # Stream MJPEG (best-effort)
                for _ in range(2000):  # ~stream for a while
                    img = _jpg_snapshot()
                    if not img:
                        time.sleep_ms(50)
                        continue
                    cl.send(b"--" + BOUNDARY.encode() + b"\r\n")
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
                cl.send(INDEX)
                cl.close()
        except Exception:
            try:
                cl.close()
            except Exception:
                pass
