@echo off
rem Double-clic : relais du robot (USB) + ouverture de l'application dans le navigateur.
rem Fermer cette fenetre (ou Ctrl+C) pour liberer le port USB, par exemple avant un televersement Arduino.
cd /d "%~dp0"
python scripts\live.py --open
pause
