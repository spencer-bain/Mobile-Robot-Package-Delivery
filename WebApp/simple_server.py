#!/usr/bin/env python
import http.server
import socketserver

class MyRequestHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.path = '/old_index.html'
        return http.server.SimpleHTTPRequestHandler.do_GET(self)


Handler = MyRequestHandler
server = socketserver.TCPServer(('10.214.152.67', 9083), Handler)    
server.serve_forever() 

