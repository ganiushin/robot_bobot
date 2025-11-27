import http.server
import ssl
import socketserver
import os
import sys

# Simple HTTPS static server for Web Bluetooth testing
# Usage:
#   python3 https_dev_server.py [port] [serve_dir]

def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8443
    serve_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(__file__), "..", "web")
    serve_dir = os.path.abspath(serve_dir)
    os.chdir(serve_dir)

    cert_file = os.path.join(os.path.dirname(__file__), "dev.cert.pem")
    key_file = os.path.join(os.path.dirname(__file__), "dev.key.pem")

    if not (os.path.exists(cert_file) and os.path.exists(key_file)):
        print("ERROR: dev cert not found. Generate with:\n"
              "  openssl req -x509 -newkey rsa:2048 -nodes -sha256 -days 365 \\\n+  -keyout tools/dev.key.pem -out tools/dev.cert.pem -subj '/CN=localhost'\n")
        sys.exit(1)

    handler = http.server.SimpleHTTPRequestHandler
    httpd = socketserver.TCPServer(("0.0.0.0", port), handler)

    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(certfile=cert_file, keyfile=key_file)
    httpd.socket = context.wrap_socket(httpd.socket, server_side=True)

    print(f"Serving HTTPS on https://localhost:{port} from {serve_dir}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()

if __name__ == "__main__":
    main()


