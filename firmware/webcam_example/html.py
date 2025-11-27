INDEX = """
HTTP/1.1 200 OK
Content-Type: text/html
Cache-Control: no-cache

<!doctype html>
<html><head><meta charset='utf-8'><title>ESP32-CAM</title></head>
<body>
<h3>ESP32-CAM MicroPython</h3>
<p><a href="/snap">Снимок</a> | <a href="/webcam">Веб-камера (MJPEG)</a></p>
<p><img id="img" src="/snap" style="max-width:100%"/></p>
<script>setInterval(()=>{document.getElementById('img').src='/snap?ts='+Date.now()},5000)</script>
</body></html>
"""
