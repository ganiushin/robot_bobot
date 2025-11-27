import network
import time


def connect_sta(ssid, password, timeout_ms=15000):
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
    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    if authmode is None:
        authmode = 0 if not password else 3
    if password:
        ap.config(essid=ssid, password=password, channel=channel, authmode=authmode)
    else:
        ap.config(essid=ssid, channel=channel, authmode=authmode)
    for _ in range(20):
        if ap.active():
            break
        time.sleep_ms(100)
    return ap.ifconfig()


def stop_ap():
    ap = network.WLAN(network.AP_IF)
    ap.active(False)
    return True


