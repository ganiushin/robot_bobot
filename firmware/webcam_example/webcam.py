import time

try:
    import camera
except ImportError:
    camera = None

try:
    from wifi import connect_wifi, start_ap
    import site
except ImportError:
    # Fallback for package usage
    from .wifi import connect_wifi, start_ap
    from . import site


def init_camera(framesize='QVGA', quality=85):
    if not camera:
        raise RuntimeError('camera module not available')
    try:
        if hasattr(camera, 'deinit'):
            try:
                camera.deinit()
            except Exception:
                pass
        ok = camera.init()
        if ok is False:
            raise RuntimeError('camera.init() returned False')
        # Set JPEG
        pf = None
        if hasattr(camera, 'PixelFormat'):
            pf = getattr(camera.PixelFormat, 'JPEG', None) or getattr(camera.PixelFormat, 'jpeg', None)
        if pf and hasattr(camera, 'reconfigure'):
            try:
                camera.reconfigure(pixel_format=pf)
            except Exception:
                pass
        # Frame size
        fs = None
        if hasattr(camera, 'FrameSize'):
            fs = getattr(camera.FrameSize, framesize, None) or getattr(camera.FrameSize, framesize.lower(), None)
        if fs is None:
            fs = getattr(camera, 'FRAME_' + framesize, None)
        if fs is not None:
            if hasattr(camera, 'framesize'):
                try:
                    camera.framesize(fs)
                except Exception:
                    pass
            if hasattr(camera, 'reconfigure'):
                try:
                    camera.reconfigure(frame_size=fs)
                except Exception:
                    pass
        # Quality
        if hasattr(camera, 'set_quality'):
            try:
                camera.set_quality(quality)
            except Exception:
                pass
        # Warm up
        for _ in range(5):
            try:
                _ = camera.capture()
            except Exception:
                pass
            time.sleep_ms(50)
    except Exception as e:
        raise


def main(ssid, password, framesize='QVGA'):
    init_camera(framesize)
    ifcfg = connect_wifi(ssid, password)
    # Start HTTP server
    site.serve()


def main_ap(ssid='ESP32-CAM', password='', framesize='QVGA', channel=6):
    init_camera(framesize)
    ifcfg = start_ap(ssid=ssid, password=password, channel=channel)
    # Serve on AP IP (defaults to 192.168.4.1)
    site.serve()


if __name__ == '__main__':
    # Replace with your WiFi credentials before uploading
    # STA example:
    # main('2.4_GHz', '47ddj4kt', framesize='QVGA')
    # AP example (open network):
    main_ap('ESP32-CAM', '', framesize='QVGA', channel=6)
