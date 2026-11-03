"""Acquisition de scans LD06 par le câble USB, sans navigateur ni copier-coller.

Usage (moniteur série de l'IDE Arduino fermé) :
    python scripts/capture.py              # trouve le port de l'ESP32 tout seul
    python scripts/capture.py --port COM9

Puis : Entrée = capturer un scan, i = infos du lidar, q = quitter.
Chaque scan est enregistré dans data/real/ (scan_01.csv, scan_02.csv, ...) et s'ouvre
ensuite dans l'application avec « Ouvrir des scans… » (ou par glisser-déposer).

Fonctionne avec les deux programmes ESP32 :
- firmware/ESP32_LD06_Capture : le scan est demandé avec 's' et reçu en entier (CSV) ;
- ESP32_LD06_Test (programme du module, Teleplot) : on enregistre les 3 tours suivants
  (lignes >lidar:...|xy). L'application les fusionne pour combler les trames perdues.

Dépendance : pyserial (python -m pip install pyserial).
"""

import argparse
import sys
import time
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial manquant : python -m pip install pyserial")

ROOT = Path(__file__).resolve().parent.parent
TELEPLOT_TOURS = 3


def usb_ports():
    """Ports série USB présents (les ports « Bluetooth » de Windows sont ignorés)."""
    return [p for p in list_ports.comports() if "bluetooth" not in (p.description or "").lower() and p.vid is not None]


def find_port():
    """Premier port série USB, ou arrêt du programme s'il n'y en a pas."""
    ports = usb_ports()
    if not ports:
        found = ", ".join(f"{p.device} ({p.description})" for p in list_ports.comports()) or "aucun"
        sys.exit(f"Aucun port USB trouvé (ports présents : {found}). Le robot est-il branché ?")
    if len(ports) > 1:
        print("Plusieurs ports USB :", ", ".join(f"{p.device} ({p.description})" for p in ports), "-> utilisation de", ports[0].device)
    return ports[0].device


def read_line(ser):
    return ser.readline().decode("utf-8", errors="replace").strip()


def detect_mode(ser, wait=4.0):
    """'capture' (programme de capture) ou 'teleplot' (programme de test), d'après ce que l'ESP32 envoie."""
    t0 = time.time()
    asked = False
    while time.time() - t0 < wait:
        line = read_line(ser)
        if line.startswith(">") and "|xy" in line:
            return "teleplot"
        if line.startswith("# LD06 ready") or line.startswith("# info"):
            return "capture"
        if not asked and time.time() - t0 > 1.5:  # l'ESP32 n'a peut-être pas redémarré : on l'interroge
            ser.write(b"i")
            asked = True
    return None


def capture_block(ser, timeout=6.0):
    """Programme de capture : envoie 's' et renvoie le bloc complet (lignes), ou lève une erreur."""
    ser.reset_input_buffer()
    ser.write(b"s")
    lines = []
    t0 = time.time()
    while time.time() - t0 < timeout:
        line = read_line(ser)
        if not line:
            continue
        if line.startswith("# error"):
            raise RuntimeError(line[2:])
        if line.startswith("# LD06 scan"):
            lines = [line]
        elif lines:
            lines.append(line)
            if line.startswith("# end"):
                return lines
    raise RuntimeError("pas de réponse complète de l'ESP32 en 6 s")


def capture_teleplot(ser, tours=TELEPLOT_TOURS, timeout=8.0):
    """Programme de test : renvoie les `tours` prochaines lignes Teleplot complètes."""
    ser.reset_input_buffer()
    read_line(ser)  # ligne en cours, probablement coupée par le vidage du tampon
    lines = []
    t0 = time.time()
    while len(lines) < tours and time.time() - t0 < timeout:
        line = read_line(ser)
        if line.startswith(">") and line.endswith("|xy"):
            lines.append(line)
    if not lines:
        raise RuntimeError("aucune ligne >lidar: reçue (le lidar tourne-t-il ?)")
    return lines


def summary(lines, mode):
    """Nombre de points valides (distance > 0) dans le bloc."""
    if mode == "teleplot":
        counts = [sum(1 for p in l.split(":", 1)[1].split("|")[0].split(";") if p and p != "0:0") for l in lines]
        return f"{len(lines)} tours, {min(counts)} à {max(counts)} points par tour"
    rows = [l.split(",") for l in lines if l[:1].isdigit()]

    def hit(r):
        try:
            return float(r[1]) > 0
        except (IndexError, ValueError):
            return False

    valid = sum(1 for r in rows if hit(r))
    return f"{len(rows)} points, dont {valid} avec un impact"


def next_name(out, ext):
    k = 1
    while any((out / f"scan_{k:02d}{e}").exists() for e in (".csv", ".txt")):
        k += 1
    return out / f"scan_{k:02d}{ext}"


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", help="port série de l'ESP32 (ex. COM9) ; détecté automatiquement sinon")
    ap.add_argument("--out", default=str(ROOT / "data" / "real"), help="dossier des scans (défaut : data/real)")
    args = ap.parse_args()

    port = args.port or find_port()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    try:
        ser = serial.Serial(port, 115200, timeout=0.2)
    except serial.SerialException as err:
        sys.exit(f"Impossible d'ouvrir {port} : {err}\n-> Fermer le moniteur série de l'IDE Arduino (un seul programme à la fois).")

    print(f"Connecté à {port}, attente de l'ESP32…")
    mode = detect_mode(ser)
    if mode is None:
        sys.exit("L'ESP32 ne répond pas. Téléverser ESP32_LD06_Capture ou ESP32_LD06_Test, puis relancer.")
    print("Programme détecté :", "capture à la demande (ESP32_LD06_Capture)" if mode == "capture" else "programme de test (Teleplot)")
    if mode == "capture":
        ser.reset_input_buffer()
        ser.write(b"i")
        print(read_line(ser) or read_line(ser))
    print(f"Scans enregistrés dans {out}")
    print("Poser le robot, ne plus le toucher, puis Entrée pour capturer. i = infos, q = quitter.\n")

    while True:
        cmd = input("> ").strip().lower()
        if cmd == "q":
            break
        if cmd == "i" and mode == "capture":
            ser.reset_input_buffer()
            ser.write(b"i")
            print(read_line(ser) or read_line(ser))
            continue
        try:
            if mode == "capture":
                lines = capture_block(ser)
                path = next_name(out, ".csv")
            else:
                lines = capture_teleplot(ser)
                path = next_name(out, ".txt")
        except RuntimeError as err:
            print("Échec :", err)
            continue
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        print(f"{path.name} : {summary(lines, mode)}")
    ser.close()


if __name__ == "__main__":
    main()
