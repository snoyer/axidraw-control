import os
import shutil
import logging
import threading
import signal
from http.server import HTTPServer
from http.server import BaseHTTPRequestHandler
import json
import argparse

from axidrawcontrol.axidraw import Axidraw2, EBBSerialError


axidraw = None
axidraw_config = dict(
    pen_positions = [1,.5,.4],
    speed_settings = (.8,.6,.5),
)
axidraw_lock = threading.Lock()

class RequestHandler(BaseHTTPRequestHandler):
    
    def do_GET(self):
        logging.debug('req: %s', self.path)

        routes = {
            '/': 'index.html'
        }

        path = routes.get(self.path, self.path)
        abs_path = os.path.join(os.path.dirname(__file__), path.lstrip('/'))

        if os.path.isfile(abs_path):
            self.send_file(abs_path)
        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write('not found'.encode())


    def do_POST(self):
        global axidraw, axidraw_config

        def update_config(update):
            if update:
                axidraw_config.update(update)
            return axidraw_config

        def move_pen(pos):
            with axidraw_lock:
                axidraw.move_pen(pos)

        def draw(many_xyzs):
            speed_settings = axidraw_config['speed_settings']
            pen_positions = axidraw_config['pen_positions']
            pen_positions.sort(reverse=True)
            
            def map_xyz(xyz):
                x,y = xyz[:2]
                z = xyz[2] if len(xyz)>2 else 0.5
                return x,y, pen_positions[1] + z*(pen_positions[2]-pen_positions[1])
                
            with axidraw_lock:
                for xyzs in many_xyzs:
                    xyzs = [map_xyz(xyz) for xyz in xyzs]
                    axidraw.move_pen(max(axidraw_config['pen_positions']))
                    axidraw.draw(xyzs, speed_settings)

                axidraw.move_to([(0,0)], speed_settings)
                axidraw.wait_until_stopped()
                axidraw.disable_motors()

        functions = {
            '/get-config': update_config,
            '/update-config': update_config,
            '/move-pen': move_pen,
            '/draw': draw,
        }

        func = functions.get(self.path)
        if func:
            content_length = int(self.headers['Content-Length'])
            data = self.rfile.read(content_length).decode() if content_length else None
            data = json.loads(data) if data else None


            response = func(data)

            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            if response:
                self.wfile.write(json.dumps(response).encode())
            else:
                self.wfile.write(json.dumps({}).encode())
        else:
            self.send_response(404)
            self.end_headers()



    def send_file(self, path, content_type=None):
        content_types = {
            '.js': 'text/javascript',
            '.html': 'text/html',
            '.css': 'text/css',
        }
        name,ext = os.path.splitext(path)
        self.send_response(200)
        self.send_header('Content-Type', content_type if content_type else content_types.get(ext))
        self.end_headers()
        shutil.copyfileobj(open(path, 'rb'), self.wfile)
            


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8081)
    parser.add_argument('--ebb-port')

    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG)
    logging.getLogger('axidrawcontrol.ebb').setLevel(logging.WARN)
    logging.getLogger('axidrawcontrol.motionplanning').setLevel(logging.WARN)


    httpserver = HTTPServer(('0.0.0.0', args.port), RequestHandler)
    httpserver_thread = threading.Thread(target=httpserver.serve_forever)
    logging.info('running on http://%s:%d', httpserver.server_name, httpserver.server_port)


    stop_event = threading.Event()
    def exit_gracefully(signum=None, frame=None):
        logging.debug('terminating...')

        httpserver.shutdown()
        httpserver_thread.join()
        logging.info('server shut down')
        
        stop_event.set()

    axidraw = Axidraw2(port=args.ebb_port)
    httpserver_thread.start()


    signal.signal(signal.SIGINT, exit_gracefully)
    signal.signal(signal.SIGTERM, exit_gracefully)
    try:
        stop_event.wait()
    except KeyboardInterrupt:
       exit_gracefully()
