"""Robot en temps réel : relaie les scans de l'ESP32 (câble USB) vers l'application web.

Usage (moniteur série de l'IDE Arduino fermé) :
    python scripts/live.py              # trouve le port de l'ESP32 tout seul
    python scripts/live.py --port COM9

Puis ouvrir http://localhost:8080/ : le scan du lidar s'affiche et se met à jour en continu.
Ce programme sert aussi l'application (inutile de lancer node serve.mjs en plus).

- Avec firmware/ESP32_RobotUSB (921600 bauds), les scans sont demandés en boucle ('s'),
  4 à 5 par seconde, et l'application peut piloter les moteurs : /live/cmd?c=F (F avancer,
  B reculer, G / D glisser, Q / E tourner, X arrêt, V120 vitesse), transmis tels quels.
- Avec firmware/ESP32_LD06_Capture (115200 bauds), les scans sont demandés en boucle ('s'),
  environ un par seconde, complets.
- Avec le programme de test du module (Teleplot), chaque tour reçu est relayé tel quel
  (environ deux par seconde, avec les trous dus aux trames perdues par ce programme).
Si le câble est débranché ou le port occupé, le relais réessaie tout seul toutes les 2 s.

Dépendance : pyserial (python -m pip install pyserial).
"""

import argparse
import json
import sys
import threading
import time
import webbrowser
from functools import partial
from urllib.parse import parse_qs, urlparse
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from capture import ROOT, capture_block, detect_mode, read_line, serial, usb_ports  # noqa: E402

MODES = {"capture": "programme de capture", "teleplot": "programme de test (Teleplot)"}
BAUDS = (921600, 115200)  # ESP32_RobotUSB, puis ESP32_LD06_Capture et le programme de test
COMMAND_CHARS = set("FBGDQEXV0123456789")


class Hub:
    """Dernier scan et dernier état du robot, partagés entre le fil série et les navigateurs."""

    def __init__(self):
        self.cond = threading.Condition()
        self.scan_seq = 0
        self.scan = None
        self.status_seq = 0
        self.status = {"connected": False, "message": "Démarrage…"}
        self.ser = None  # port série ouvert, pour les ordres moteurs de l'application
        self.write_lock = threading.Lock()

    def send_command(self, cmd):
        """Ordre moteur de l'application vers l'ESP32 ; False si le robot n'est pas branché."""
        ser = self.ser
        if ser is None:
            return False
        with self.write_lock:
            ser.write(cmd.encode("ascii"))
        return True

    def publish_scan(self, text, mode):
        with self.cond:
            self.scan_seq += 1
            self.scan = {"seq": self.scan_seq, "mode": mode, "time": time.time(), "text": text}
            self.cond.notify_all()

    def publish_status(self, message, connected=False, **extra):
        with self.cond:
            self.status_seq += 1
            self.status = {"connected": connected, "message": message, **extra}
            self.cond.notify_all()
        print(message)


def serial_worker(hub, port_arg):
    """Lit l'ESP32 en continu ; se reconnecte tout seul après une coupure."""
    while True:
        ports = [port_arg] if port_arg else [p.device for p in usb_ports()]
        if not ports:
            hub.publish_status("Robot non branché : aucun port USB trouvé")
            time.sleep(2)
            continue
        port = ports[0]
        try:
            mode = None
            for baud in BAUDS:
                with serial.Serial(port, baud, timeout=0.2) as ser:
                    hub.publish_status(f"{port} ouvert à {baud} bauds, attente de l'ESP32…")
                    mode = detect_mode(ser, wait=3.0)
                    if mode is None:
                        continue
                    hub.publish_status(f"Robot connecté sur {port} ({baud} bauds) : {MODES[mode]}", connected=True, port=port, mode=mode, baud=baud)
                    hub.ser = ser
                    try:
                        if mode == "capture":
                            while True:
                                try:
                                    with hub.write_lock:
                                        ser.reset_input_buffer()
                                        ser.write(b"s")
                                    hub.publish_scan("\n".join(capture_block(ser, send=False)), mode)
                                except RuntimeError as err:
                                    hub.publish_status(f"{port} : {err}", connected=True, port=port, mode=mode, baud=baud)
                                    time.sleep(0.5)
                        else:
                            while True:
                                line = read_line(ser)
                                if line.startswith(">") and line.endswith("|xy"):
                                    hub.publish_scan(line, mode)  # le dernier tour, tel quel
                    finally:
                        hub.ser = None
            if mode is None:
                hub.publish_status(f"{port} : l'ESP32 ne répond pas (programme téléversé ?)")
                time.sleep(2)
        except serial.SerialException as err:
            busy = "denied" in str(err).lower() or "refusé" in str(err).lower() or "PermissionError" in str(err)
            hint = "port occupé : fermer le moniteur série de l'IDE Arduino" if busy else "liaison perdue"
            hub.publish_status(f"{port} : {hint}, nouvel essai dans 2 s")
            time.sleep(2)


class Handler(SimpleHTTPRequestHandler):
    """Fichiers de l'application + flux /live (Server-Sent Events) + état /live/status."""

    # Sous Windows, le registre peut associer .js à text/plain : les modules ne se chargeraient pas.
    extensions_map = {
        **SimpleHTTPRequestHandler.extensions_map,
        ".js": "text/javascript",
        ".mjs": "text/javascript",
        ".css": "text/css",
        ".json": "application/json",
        ".csv": "text/csv",
        ".svg": "image/svg+xml",
    }

    def __init__(self, *args, hub=None, **kwargs):
        self.hub = hub
        super().__init__(*args, **kwargs)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, fmt, *args):
        pass  # pas de journal de chaque requête dans la console

    def do_GET(self):
        path = self.path.split("?")[0]
        if path in ("/", "/app"):
            self.send_response(302)
            self.send_header("Location", "/app/")
            self.end_headers()
        elif path == "/live/status":
            body = json.dumps(self.hub.status).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif path == "/live":
            self.stream()
        elif path == "/live/cmd":
            # ordre moteur de l'application : F, B, G, D, Q, E, X ou V<vitesse>
            cmd = parse_qs(urlparse(self.path).query).get("c", [""])[0]
            ok = bool(cmd) and set(cmd) <= COMMAND_CHARS and len(cmd) <= 5
            sent = ok and self.hub.send_command(cmd + ("\n" if cmd.startswith("V") else ""))
            body = b"OK" if sent else (b"robot non branche" if ok else b"commande invalide")
            self.send_response(200 if sent else (503 if ok else 400))
            self.send_header("Content-Type", "text/plain")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            super().do_GET()

    def stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        hub = self.hub
        sent_scan, sent_status = -1, -1
        try:
            while True:
                with hub.cond:
                    hub.cond.wait_for(lambda: hub.scan_seq != sent_scan or hub.status_seq != sent_status, timeout=15)
                    status, status_seq, scan, scan_seq = hub.status, hub.status_seq, hub.scan, hub.scan_seq
                out = ""
                if status_seq != sent_status:
                    out += f"event: status\ndata: {json.dumps(status)}\n\n"
                    sent_status = status_seq
                if scan is not None and scan_seq != sent_scan:
                    out += f"event: scan\ndata: {json.dumps(scan)}\n\n"
                    sent_scan = scan_seq
                self.wfile.write((out or ": ping\n\n").encode())
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass  # onglet fermé


class Server(ThreadingHTTPServer):
    daemon_threads = True
    # Sous Windows, SO_REUSEADDR laisse un second relais s'installer sur le même port HTTP :
    # les deux se disputeraient alors le port série. Sans cette option, le second s'arrête.
    allow_reuse_address = False

    def handle_error(self, request, client_address):
        # Le navigateur ferme souvent ses connexions en cours de route (rechargement, onglet
        # fermé) : sans gravité, inutile d'afficher une trace d'erreur.
        if isinstance(sys.exc_info()[1], ConnectionError):
            return
        super().handle_error(request, client_address)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", help="port série de l'ESP32 (ex. COM9) ; détecté automatiquement sinon")
    ap.add_argument("--http", type=int, default=8080, help="port de l'application web (défaut 8080)")
    ap.add_argument("--open", action="store_true", help="ouvrir l'application dans le navigateur")
    args = ap.parse_args()

    hub = Hub()
    threading.Thread(target=serial_worker, args=(hub, args.port), daemon=True).start()
    try:
        server = Server(("localhost", args.http), partial(Handler, hub=hub, directory=str(ROOT)))
    except OSError:
        sys.exit(f"Le port {args.http} est déjà utilisé (node serve.mjs encore lancé ?). Le fermer ou ajouter --http 8081.")
    print(f"Application : http://localhost:{args.http}/  (Ctrl+C pour arrêter)")
    if args.open:
        threading.Timer(1.0, webbrowser.open, [f"http://localhost:{args.http}/"]).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
