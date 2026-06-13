from http.server import HTTPServer, BaseHTTPRequestHandler
import time

class SSEHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        try:
            for i in range(5):
                self.wfile.write(f"data: {{\"count\": {i}}}\n\n".encode())
                self.wfile.flush()
                time.sleep(0.5)
        except:
            pass

if __name__ == '__main__':
    server = HTTPServer(('127.0.0.1', 8080), SSEHandler)
    server.timeout = 3.0
    server.handle_request()
