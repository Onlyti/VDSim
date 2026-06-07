from http.server import BaseHTTPRequestHandler
from urllib.parse import parse_qs, urlparse

from api.responses import json_response
from api.routes import ApiContext, handle_get, handle_post, parse_post_body


def make_handler(ctx: ApiContext):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _json(self, obj, code=200):
            json_response(self, obj, code)

        def do_GET(self):
            route, qs = urlparse(self.path).path, parse_qs(urlparse(self.path).query)
            if not handle_get(self, route, qs, ctx):
                self.send_error(404)

        def do_POST(self):
            n = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(n) or b"{}"
            ct = self.headers.get("Content-Type", "")
            path = urlparse(self.path).path
            body = parse_post_body(self, raw, ct, path)
            if not handle_post(self, path, body, ctx):
                self.send_error(404)

    return Handler
