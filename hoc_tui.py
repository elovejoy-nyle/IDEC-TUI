#!/usr/bin/env python3
"""
Interactive terminal shell for controlling an IDEC PLC over serial using MiSmSerial.

Commands:
  clear                  Clear screen
  config                 Configure serial connection settings
  connect                Open the serial connection using current config
  disconnect             Close the serial connection
  status                 Show current config + connection state
  help                   Show help
  methods                List supported MiSmSerial methods
  q | quit | exit        Quit

PLC commands can be entered directly after connecting, for example:
  read D0100
  write D0100 1234
  read_bit M8004.15
  write_bit M8004.15 1
  input I0
  output Q0 1
  read_float D0200
  write_float D0200 12.5
  read_timer 0 4
  read_error

Notes:
- This app expects MiSmSerial.py to be importable from the same directory,
  or installed somewhere on your PYTHONPATH.
- MiSmSerial supports methods including read/write, read_bit/write_bit,
  input/output, read_float/write_float, read_timer, write_counter,
  and read_error.
"""

from __future__ import annotations

import ast
import json
import os
import shlex
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

try:
    from MiSmSerial import MiSmSerial
except ImportError:
    print(
        "Could not import MiSmSerial. Put this script next to MiSmSerial.py "
        "or install the library so Python can import it.",
        file=sys.stderr,
    )
    raise


CONFIG_PATH = Path.home() / ".plc_terminal_config.json"


@dataclass
class AppConfig:
    port: str = "/dev/ttyUSB0"
    device: str = "FF"
    baud: int = 9600
    timeout: float = 1.0
    bytesize: int = 8
    parity: str = "N"
    stopbits: int = 1
    debug: bool = False
    bcc_mode: str = "auto"


class PLCTerminalApp:
    def __init__(self) -> None:
        self.config = self._load_config()
        self.plc: MiSmSerial | None = None
        self.running = True

    def _load_config(self) -> AppConfig:
        if CONFIG_PATH.exists():
            try:
                data = json.loads(CONFIG_PATH.read_text())
                return AppConfig(**data)
            except Exception as exc:
                print(f"Warning: failed to load config: {exc}")
        return AppConfig()

    def _save_config(self) -> None:
        CONFIG_PATH.write_text(json.dumps(asdict(self.config), indent=2))

    def prompt(self) -> str:
        state = "connected" if self.plc else "disconnected"
        return f"plc[{state}]> "

    def run(self) -> None:
        print("PLC Terminal")
        print("Type 'help' for help, 'config' to configure the port, 'q' to quit.")
        while self.running:
            try:
                line = input(self.prompt()).strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if not line:
                continue

            try:
                self.handle_line(line)
            except Exception as exc:
                print(f"Error: {exc}")

        self.disconnect()
        print("Bye.")


    def handle_line(self, line: str) -> None:
        parts = shlex.split(line)
        cmd = parts[0].lower()
        args = parts[1:]

        if cmd in {"q", "quit", "exit"}:
            self.running = False
            return
        if cmd == "help":
            self.show_help()
            return
        if cmd == "methods":
            self.show_methods()
            return
        if cmd == "config":
            self.configure_interactive()
            return
        if cmd == "connect":
            self.connect()
            return
        if cmd == "disconnect":
            self.disconnect()
            return
        if cmd == "status":
            self.show_status()
            return
        if cmd == "clear":
            os.system('clear') #nix only for now
            return      
        if self.plc is None:
            print("Not connected. Run 'config' and then 'connect' first.")
            return

        self.execute_plc_command(cmd, args)

    def show_help(self) -> None:
        print(
            """
Built-in commands:
  clear                  Clear screen
  config                 Configure connection settings interactively
  connect                Open serial connection
  disconnect             Close serial connection
  status                 Show config and connection status
  methods                List supported MiSmSerial methods
  help                   Show this help
  q | quit | exit        Quit

PLC commands:
  read <addr>
  write <addr> <value>
  read_bit <addr>
  write_bit <addr> <0|1>
  input <I0|X0000|0>
  output <Q0|Y0000|0> <0|1>
  read_float <addr> [endian]
  write_float <addr> <value> [endian]
  read_timer <timer_num> [count]
  write_counter <counter_num> <preset>
  read_error [addr] [nbytes]

Examples:
  config
  connect
  read D0100
  write D0100 42
  read_bit M8004.15
  write_bit M8004.15 1
  input I0
  output Q0 1
  read_float D0200
  write_float D0200 12.5
  read_timer 0 4
  read_error
""".strip()
        )

    def show_methods(self) -> None:
        methods = [
            "read(addr, endian=0, dtype=None)",
            "write(addr, value, endian=0, dtype=None)",
            "read_bit(addr, endian=0, dtype=None)",
            "write_bit(addr, on, endian=0, dtype=None)",
            "input(bit)",
            "output(bit, on=1)",
            "read_float(addr, endian=0, dtype=None)",
            "write_float(addr, value, endian=0, dtype=None)",
            "read_timer(tnum, count=1)",
            "write_counter(cnum, preset)",
            "read_error(addr=0, nbytes=12)",
            "close()",
        ]
        print("Supported MiSmSerial methods:")
        for item in methods:
            print(f"  - {item}")

    def show_status(self) -> None:
        print("Connection status:", "connected" if self.plc else "disconnected")
        print(json.dumps(asdict(self.config), indent=2))

    def configure_interactive(self) -> None:
        print("Press Enter to keep the current value shown in [brackets].")
        self.disconnect()

        self.config.port = self.ask_str("Port", self.config.port)
        self.config.device = self.ask_str("Device", self.config.device).upper()
        self.config.baud = self.ask_int("Baud", self.config.baud)
        self.config.timeout = self.ask_float("Timeout (seconds)", self.config.timeout)
        self.config.bytesize = self.ask_int("Bytesize", self.config.bytesize)
        self.config.parity = self.ask_str("Parity", self.config.parity).upper()
        self.config.stopbits = self.ask_int("Stopbits", self.config.stopbits)
        self.config.debug = self.ask_bool("Debug", self.config.debug)
        self.config.bcc_mode = self.ask_choice(
            "BCC mode", self.config.bcc_mode, ["auto", "enq", "no_enq"]
        )

        self._save_config()
        print(f"Saved config to {CONFIG_PATH}")

    def ask_str(self, label: str, default: str) -> str:
        value = input(f"{label} [{default}]: ").strip()
        return value or default

    def ask_int(self, label: str, default: int) -> int:
        value = input(f"{label} [{default}]: ").strip()
        return default if value == "" else int(value)

    def ask_float(self, label: str, default: float) -> float:
        value = input(f"{label} [{default}]: ").strip()
        return default if value == "" else float(value)

    def ask_bool(self, label: str, default: bool) -> bool:
        default_text = "y" if default else "n"
        value = input(f"{label} [y/n, default {default_text}]: ").strip().lower()
        if value == "":
            return default
        return value in {"y", "yes", "1", "true", "on"}

    def ask_choice(self, label: str, default: str, choices: list[str]) -> str:
        value = input(f"{label} {choices} [{default}]: ").strip().lower()
        if value == "":
            return default
        if value not in choices:
            raise ValueError(f"Expected one of: {', '.join(choices)}")
        return value

    def connect(self) -> None:
        self.disconnect()
        self.plc = MiSmSerial(
            port=self.config.port,
            device=self.config.device,
            baud=self.config.baud,
            timeout=self.config.timeout,
            bytesize=self.config.bytesize,
            parity=self.config.parity,
            stopbits=self.config.stopbits,
            debug=self.config.debug,
            bcc_mode=self.config.bcc_mode,
        )
        print(
            f"Connected to {self.config.port} "
            f"(baud={self.config.baud}, device={self.config.device}, bcc_mode={self.config.bcc_mode})"
        )

    def disconnect(self) -> None:
        if self.plc is not None:
            try:
                self.plc.close()
            finally:
                self.plc = None
                print("Disconnected.")

    def execute_plc_command(self, cmd: str, args: list[str]) -> None:
        assert self.plc is not None

        if cmd == "read":
            self.require_args(cmd, args, 1)
            print(self.plc.read(args[0]))
            return

        if cmd == "write":
            self.require_args(cmd, args, 2)
            value = self.parse_value(args[1])
            print(self.plc.write(args[0], int(value)))
            return

        if cmd == "read_bit":
            self.require_args(cmd, args, 1)
            print(self.plc.read_bit(args[0]))
            return

        if cmd == "write_bit":
            self.require_args(cmd, args, 2)
            print(self.plc.write_bit(args[0], int(self.parse_value(args[1]))))
            return

        if cmd == "input":
            self.require_args(cmd, args, 1)
            io_arg = self.parse_io_arg(args[0])
            print(self.plc.input(io_arg))
            return

        if cmd == "output":
            self.require_args(cmd, args, 2)
            io_arg = self.parse_io_arg(args[0])
            print(self.plc.output(io_arg, int(self.parse_value(args[1]))))
            return

        if cmd == "read_float":
            self.require_args(cmd, args, 1)
            endian = int(self.parse_value(args[1])) if len(args) > 1 else 0
            print(self.plc.read_float(args[0], endian=endian))
            return

        if cmd == "write_float":
            self.require_args(cmd, args, 2)
            value = float(self.parse_value(args[1]))
            endian = int(self.parse_value(args[2])) if len(args) > 2 else 0
            print(self.plc.write_float(args[0], value, endian=endian))
            return

        if cmd == "read_timer":
            tnum = int(self.parse_value(args[0])) if args else 0
            count = int(self.parse_value(args[1])) if len(args) > 1 else 1
            print(self.plc.read_timer(tnum, count))
            return

        if cmd == "write_counter":
            self.require_args(cmd, args, 2)
            cnum = int(self.parse_value(args[0]))
            preset = int(self.parse_value(args[1]))
            print(self.plc.write_counter(cnum, preset))
            return

        if cmd == "read_error":
            addr = int(self.parse_value(args[0])) if len(args) > 0 else 0
            nbytes = int(self.parse_value(args[1])) if len(args) > 1 else 12
            print(self.plc.read_error(addr, nbytes))
            return

        if hasattr(self.plc, cmd):
            method = getattr(self.plc, cmd)
            parsed_args = [self.parse_value(a) for a in args]
            result = method(*parsed_args)
            if result is not None:
                print(result)
            return

        print(f"Unknown command: {cmd}. Type 'help' or 'methods'.")

    def require_args(self, cmd: str, args: list[str], min_count: int) -> None:
        if len(args) < min_count:
            raise ValueError(f"{cmd} needs at least {min_count} argument(s)")

    def parse_io_arg(self, raw: str) -> Any:
        return int(raw) if raw.isdigit() else raw

    def parse_value(self, raw: str) -> Any:
        text = raw.strip()

        if text.lower() in {"true", "false"}:
            return text.lower() == "true"

        try:
            if text.startswith("0x") or text.startswith("0X"):
                return int(text, 16)
            if text.startswith("0b") or text.startswith("0B"):
                return int(text, 2)
            if text.startswith("0o") or text.startswith("0O"):
                return int(text, 8)
        except ValueError:
            pass

        try:
            return ast.literal_eval(text)
        except Exception:
            return text


def main() -> int:
    app = PLCTerminalApp()
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
