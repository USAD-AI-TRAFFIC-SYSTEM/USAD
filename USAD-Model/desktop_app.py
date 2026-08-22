"""USAD Desktop Application Launcher.

Starts the FastAPI server and opens the React UI in the default browser.
For production distribution, this can be wrapped with pywebview for a native window.
"""

import os
import sys
import time
import threading
import webbrowser
from pathlib import Path

# Ensure USAD-Model is on sys.path
_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))


def main():
    HOST = "127.0.0.1"
    PORT = 8000
    URL = f"http://{HOST}:{PORT}"

    print("=" * 70)
    print("USAD - Urban Smart Adaptive Dispatcher")
    print("Modern Desktop UI Launcher")
    print("=" * 70)

    # Try pywebview first (native window), fall back to browser
    use_webview = False
    try:
        import webview  # noqa: F401
        use_webview = True
    except ImportError:
        pass

    if use_webview:
        # Start uvicorn in a background thread, then open pywebview window
        def _run_server():
            import uvicorn
            uvicorn.run(
                "server:app",
                host=HOST,
                port=PORT,
                log_level="warning",
                reload=False,
            )

        server_thread = threading.Thread(target=_run_server, daemon=True)
        server_thread.start()

        # Wait for server to be ready
        import urllib.request
        for _ in range(60):
            try:
                urllib.request.urlopen(f"{URL}/api/logs/summary", timeout=1)
                break
            except Exception:
                time.sleep(0.25)

        # Find icon for window
        ico_path = None
        for candidate in [
            _THIS_DIR.parent / "Logo 1.ico",
            _THIS_DIR / "assets" / "Logo 1.ico",
        ]:
            if candidate.exists():
                ico_path = str(candidate)
                break

        print(f"[Desktop] Opening native window -> {URL}")
        window = webview.create_window(
            "USAD (Urban Smart Adaptive Dispatcher)",
            URL,
            width=1440,
            height=900,
            resizable=True,
            text_select=False,
        )
        webview.start()

    else:
        # No pywebview — open in default browser
        def _open_browser():
            time.sleep(2.0)
            print(f"[Desktop] Opening browser -> {URL}")
            webbrowser.open(URL)

        threading.Thread(target=_open_browser, daemon=True).start()

        import uvicorn
        uvicorn.run(
            "server:app",
            host=HOST,
            port=PORT,
            log_level="info",
            reload=False,
        )


if __name__ == "__main__":
    main()
