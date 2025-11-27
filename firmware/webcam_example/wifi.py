import network
import time


def connect_wifi(ssid, password, timeout_ms=15000):
    sta = network.WLAN(network.STA_IF)
    sta.active(True)
    if not sta.isconnected():
        sta.connect(ssid, password)
        t0 = time.ticks_ms()
        while not sta.isconnected():
            if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
                raise OSError('WiFi connect timeout')
            time.sleep_ms(200)
    return sta.ifconfig()


def start_ap(ssid='ESP32-CAM', password='', channel=6, authmode=None):
    """Start ESP32 as Access Point. If password is empty, use open auth.
    Returns ifconfig tuple (ip, subnet, gw, dns).
    """
    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    # Determine authmode
    if authmode is None:
        # 0=OPEN, 3=WPA2-PSK typically in MicroPython
        authmode = 0 if not password else 3
    if password:
        ap.config(essid=ssid, password=password, channel=channel, authmode=authmode)
    else:
        ap.config(essid=ssid, channel=channel, authmode=authmode)
    # Wait until active
    for _ in range(20):
        if ap.active():
            break
        time.sleep_ms(100)
    return ap.ifconfig()
