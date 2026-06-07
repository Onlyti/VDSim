import json


def json_response(h, obj, code=200):
    body = json.dumps(obj).encode()
    h.send_response(code)
    h.send_header("Content-Type", "application/json")
    h.send_header("Content-Length", str(len(body)))
    h.end_headers()
    h.wfile.write(body)


def bytes_response(h, data, content_type, code=200, extra_headers=None):
    h.send_response(code)
    h.send_header("Content-Type", content_type)
    if extra_headers:
        for k, v in extra_headers.items():
            h.send_header(k, v)
    h.send_header("Content-Length", str(len(data)))
    h.end_headers()
    h.wfile.write(data)
