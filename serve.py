#!/usr/bin/env python

import os, sys
from SimpleHTTPServer import SimpleHTTPRequestHandler
import BaseHTTPServer
import subprocess

import threading, time

def openURL():
    time.sleep(1)
    print "Opening Simni URL with open ..."
    #call(["open", '"http://127.0.0.1:8000"'])
    filepath = "http://127.0.0.1:8000/simulator.html"

    if sys.platform.startswith('darwin'):
        subprocess.call(('open', filepath))
    elif os.name == 'nt':
        os.startfile(filepath)
    elif os.name == 'posix':
        subprocess.call(('xdg-open', filepath))

def test(HandlerClass=SimpleHTTPRequestHandler,
         ServerClass=BaseHTTPServer.HTTPServer):

    protocol = "HTTP/1.0"
    host = '127.0.0.1'
    port = 8000
    if len(sys.argv) > 1:
        arg = sys.argv[1]
        if ':' in arg:
            host, port = arg.split(':')
            port = int(port)
        else:
            try:
                port = int(sys.argv[1])
            except:
                host = sys.argv[1]

    server_address = (host, port)

    HandlerClass.protocol_version = protocol
    httpd = ServerClass(server_address, HandlerClass)

    threading.Thread(target=openURL).start()
    sa = httpd.socket.getsockname()
    print "Serving HTTP on", sa[0], "port", sa[1], "..."
    httpd.serve_forever()

if __name__ == "__main__":
    test()
