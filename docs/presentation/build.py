#!/usr/bin/env python3
"""
ReloBot Presentation Compiler & Dev Server
Assembles modular HTML, CSS, JS, and slide partials into a standalone, portable index.html.

Usage:
    python3 build.py              # Single build -> index.html
    python3 build.py --watch      # Watch src/ for changes & rebuild automatically
    python3 build.py --serve      # Start local HTTP server on http://localhost:8000
    python3 build.py --watch --serve
"""

import os
import sys
import time
import glob
import re
import argparse
import http.server
import socketserver
import threading

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_DIR = os.path.join(BASE_DIR, 'src')
OUTPUT_FILE = os.path.join(BASE_DIR, 'index.html')

CSS_FILES = [
    'variables.css',
    'base.css',
    'components.css',
    'slides.css'
]

JS_FILES = [
    'notes.js',
    'presentation.js'
]


def build() -> bool:
    """Compiles src/ files into docs/presentation/index.html."""
    start_time = time.time()

    template_path = os.path.join(SRC_DIR, 'index.template.html')
    if not os.path.exists(template_path):
        print(f"[ERROR] Template file not found: {template_path}")
        return False

    with open(template_path, 'r', encoding='utf-8') as f:
        template = f.read()

    # 1. Gather CSS
    css_chunks = []
    for css_file in CSS_FILES:
        path = os.path.join(SRC_DIR, 'css', css_file)
        if os.path.exists(path):
            with open(path, 'r', encoding='utf-8') as f:
                css_chunks.append(f"    /* --- {css_file} --- */\n{f.read()}")
        else:
            print(f"[WARN] CSS file not found: {path}")

    combined_css = "\n\n".join(css_chunks)
    css_injection = f"  <style>\n{combined_css}\n  </style>"

    # 2. Gather Slides
    slide_files = sorted(glob.glob(os.path.join(SRC_DIR, 'slides', '*.html')))
    if not slide_files:
        print("[WARN] No slide files found in src/slides/")
    slide_chunks = []
    for slide_file in slide_files:
        with open(slide_file, 'r', encoding='utf-8') as f:
            content = f.read().strip()
            slide_chunks.append(f"    {content}")

    combined_slides = "\n\n".join(slide_chunks)

    # 3. Gather JS
    js_chunks = []
    for js_file in JS_FILES:
        path = os.path.join(SRC_DIR, 'js', js_file)
        if os.path.exists(path):
            with open(path, 'r', encoding='utf-8') as f:
                js_chunks.append(f"    // --- {js_file} ---\n{f.read()}")
        else:
            print(f"[WARN] JS file not found: {path}")

    combined_js = "\n\n".join(js_chunks)
    js_injection = f"  <script>\n{combined_js}\n  </script>"

    # 4. Inject into template
    output = template.replace('<!-- INJECT:CSS -->', css_injection)
    output = output.replace('<!-- INJECT:SLIDES -->', combined_slides)
    output = output.replace('<!-- INJECT:JS -->', js_injection)

    # 5. Write index.html
    with open(OUTPUT_FILE, 'w', encoding='utf-8') as f:
        f.write(output)

    elapsed_ms = (time.time() - start_time) * 1000
    file_size_kb = os.path.getsize(OUTPUT_FILE) / 1024
    print(f"[BUILD SUCCESS] Generated {OUTPUT_FILE} ({len(slide_files)} slides, {file_size_kb:.1f} KB) in {elapsed_ms:.1f}ms")
    return True


def get_source_mtimes() -> dict:
    """Returns mapping of all watched files to their last modified timestamp."""
    mtimes = {}
    for root, _, files in os.walk(SRC_DIR):
        for f in files:
            path = os.path.join(root, f)
            try:
                mtimes[path] = os.path.getmtime(path)
            except OSError:
                pass
    return mtimes


def watch_and_rebuild(interval: float = 0.5):
    """Watches src/ for file changes and triggers rebuild."""
    print(f"[WATCH] Watching for changes in {SRC_DIR}... (Press Ctrl+C to stop)")
    last_mtimes = get_source_mtimes()
    try:
        while True:
            time.sleep(interval)
            current_mtimes = get_source_mtimes()
            modified = False
            for path, mtime in current_mtimes.items():
                if path not in last_mtimes or mtime > last_mtimes[path]:
                    rel_path = os.path.relpath(path, BASE_DIR)
                    print(f"\n[CHANGED] {rel_path}")
                    modified = True
                    break
            if not modified and len(current_mtimes) != len(last_mtimes):
                print(f"\n[CHANGED] File added or removed in src/")
                modified = True

            if modified:
                build()
                last_mtimes = current_mtimes
    except KeyboardInterrupt:
        print("\n[WATCH] Stopped watcher.")


def serve_http(port: int = 8000):
    """Serves the presentation directory on a local HTTP port."""
    class Handler(http.server.SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=BASE_DIR, **kwargs)

    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("", port), Handler) as httpd:
        print(f"[SERVE] Presentation live at http://localhost:{port}/index.html")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\n[SERVE] Stopped server.")


def main():
    parser = argparse.ArgumentParser(description="ReloBot Presentation Builder")
    parser.add_argument('--watch', action='store_true', help="Watch src/ directory for changes and rebuild")
    parser.add_argument('--serve', action='store_true', help="Serve directory on local HTTP server")
    parser.add_argument('--port', type=int, default=8000, help="Port for HTTP server (default: 8000)")
    args = parser.parse_args()

    # Initial build
    success = build()
    if not success:
        sys.exit(1)

    if args.serve and args.watch:
        server_thread = threading.Thread(target=serve_http, args=(args.port,), daemon=True)
        server_thread.start()
        watch_and_rebuild()
    elif args.serve:
        serve_http(args.port)
    elif args.watch:
        watch_and_rebuild()


if __name__ == '__main__':
    main()
