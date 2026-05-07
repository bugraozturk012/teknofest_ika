#!/usr/bin/env python3
"""
gcs_terminal.py — Yer Kontrol İstasyonu Terminal Arayüzü
=========================================================
Yarışmada dizüstünde çalışır. LR02 → USB-TTL → bu script.

Kullanım:
  python3 gcs_terminal.py --port /dev/ttyUSB0 --baud 9600

Klavye komutları:
  e → E-STOP aktifleştir
  r → E-STOP kaldır
  0 → MANUAL mod
  2 → FULL_AUTO mod
  q → çıkış
"""

import argparse
import json
import sys
import threading
import time

try:
    import serial
except ImportError:
    print('pyserial gerekli: pip3 install pyserial')
    sys.exit(1)

_MOD = {0: 'MANUAL', 1: 'SEMI  ', 2: 'AUTO  '}
_FSM_RENK = {
    'IDLE':           '\033[33m',   # sarı
    'NAVIGATE':       '\033[32m',   # yeşil
    'SHOOT_APPROACH': '\033[36m',   # cyan
    'SHOOT':          '\033[31m',   # kırmızı
    'MISSION_COMPLETE': '\033[35m', # mor
    'ERROR_RECOVERY': '\033[31m',   # kırmızı
}
_RESET = '\033[0m'
_KIRMIZI = '\033[91m'
_YESIL   = '\033[92m'


def batarya_bar(yuzde: int, genislik: int = 10) -> str:
    dolu = int(yuzde / 100 * genislik)
    bos  = genislik - dolu
    renk = _YESIL if yuzde > 30 else _KIRMIZI
    return f'{renk}[{"█" * dolu}{"░" * bos}]{_RESET} {yuzde:3d}%'


def ekrani_temizle():
    print('\033[2J\033[H', end='')


def gcs_dongusu(ser: serial.Serial):
    tampon = b''
    son_paket = {}

    while True:
        try:
            veri = ser.read(64)
        except serial.SerialException as exc:
            print(f'\n[HATA] Seri port: {exc}')
            break

        if not veri:
            continue

        tampon += veri
        while b'\n' in tampon:
            satir, tampon = tampon.split(b'\n', 1)
            satir = satir.strip()
            if not satir:
                continue
            try:
                son_paket = json.loads(satir.decode())
            except Exception:
                continue

            # Ekranı güncelle
            ekrani_temizle()
            p = son_paket
            mod_str  = _MOD.get(p.get('m', 0), '?')
            fsm_str  = p.get('s', '?')
            fsm_renk = _FSM_RENK.get(fsm_str, '')
            voltaj   = p.get('v', 0.0)
            batarya  = p.get('b', 0)
            e_stop   = p.get('e', 0)
            x, y     = p.get('x', 0.0), p.get('y', 0.0)
            wp       = p.get('w', 0)
            t        = p.get('t', 0)

            print('╔══════════════════════════════════════╗')
            print('║     LYDİA İKA — GCS TERMİNAL        ║')
            print('╠══════════════════════════════════════╣')
            print(f'║ Süre   : {t:>5d} s                     ║')
            print(f'║ Mod    : {mod_str:<6}                       ║')
            print(f'║ FSM    : {fsm_renk}{fsm_str:<20}{_RESET}      ║')
            print(f'║ Pozisyon: ({x:6.1f}, {y:6.1f}) m            ║')
            print(f'║ Waypoint: {wp:<3d}                          ║')
            print(f'║ Batarya: {batarya_bar(batarya):<30}║')
            print(f'║ Voltaj : {voltaj:.1f} V                       ║')
            if e_stop:
                print(f'║ {_KIRMIZI}!!! E-STOP AKTİF !!!{_RESET}              ║')
            else:
                print(f'║ E-STOP : {_YESIL}TAMAM{_RESET}                        ║')
            print('╠══════════════════════════════════════╣')
            print('║ e=E-STOP  r=Kaldır  0=Manual  2=Auto║')
            print('║ q=Çıkış                              ║')
            print('╚══════════════════════════════════════╝')
            sys.stdout.flush()


def komut_gonder(ser: serial.Serial, komut: dict):
    ham = (json.dumps(komut, separators=(',', ':')) + '\n').encode()
    try:
        ser.write(ham)
    except serial.SerialException as exc:
        print(f'[HATA] Gönderim: {exc}')


def klavye_dongusu(ser: serial.Serial):
    import tty
    import termios
    fd = sys.stdin.fileno()
    eski = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while True:
            tus = sys.stdin.read(1)
            if tus == 'e':
                komut_gonder(ser, {'cmd': 'estop', 'val': 1})
            elif tus == 'r':
                komut_gonder(ser, {'cmd': 'estop', 'val': 0})
            elif tus == '0':
                komut_gonder(ser, {'cmd': 'mode', 'val': 0})
            elif tus == '2':
                komut_gonder(ser, {'cmd': 'mode', 'val': 2})
            elif tus in ('q', '\x03'):
                print('\nÇıkılıyor...')
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, eski)


def main():
    ap = argparse.ArgumentParser(description='LYDİA GCS Terminal')
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=9600)
    args = ap.parse_args()

    print(f'LR02 bağlanıyor: {args.port} @ {args.baud}...')
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.5)
    except serial.SerialException as exc:
        print(f'Bağlantı hatası: {exc}')
        sys.exit(1)

    print('Bağlandı. Telemetri bekleniyor...')

    t = threading.Thread(target=gcs_dongusu, args=(ser,), daemon=True)
    t.start()

    try:
        klavye_dongusu(ser)
    except Exception:
        pass
    finally:
        ser.close()


if __name__ == '__main__':
    main()
