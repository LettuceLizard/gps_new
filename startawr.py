# send_cfg_robust.py
import serial, time, pathlib, sys, re

CLI_PORT = "/dev/ttyACM0"                 # your CLI / User UART
CLI_BAUD = 115200                 # typical CLI baud
CFG_PATH = r"./tdm_enet.cfg"  # <-- change this

PROMPT_RE = re.compile(rb"(?:^|\r?\n)mmw.*?[>#/]\>\s*$", re.I|re.M)
COMMENT_PREFIXES = ("%", "#", "!", "//")

def open_cli(port, baud):
    ser = serial.Serial(
        port,
        baudrate=baud,
        timeout=0.1,             # give the device time to answer
        write_timeout=1.0,
        inter_byte_timeout=0.02,
        rtscts=False,
        dsrdtr=False,
        xonxoff=False,
    )
    time.sleep(0.2)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    # Wake the prompt
    ser.write(b"\n")
    _ = read_until_prompt(ser, 2.0)
    return ser

def read_until_prompt(ser, timeout=5.0):
    end = time.time() + timeout
    buf = bytearray()
    while time.time() < end:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf.extend(chunk)
            if b"\nDone" in buf or b"\nError" in buf or PROMPT_RE.search(buf):
                # small extra drain
                t = time.time() + 0.05
                while time.time() < t:
                    more = ser.read(ser.in_waiting or 1)
                    if more: buf.extend(more)
                    else: time.sleep(0.005)
                break
        else:
            time.sleep(0.005)
    return bytes(buf)

def normalize_line(s: str) -> str:
    # drop BOM/odd whitespace and collapse spaces
    s = s.replace("\ufeff", "")
    s = " ".join(s.split())
    return s

def clean_response(response: str) -> str:
    # Clean up radar response by removing excessive whitespace and control chars
    lines = []
    for line in response.split('\n'):
        line = line.strip()
        if line and not line.isspace():
            lines.append(line)
    return '\n'.join(lines)

def send_cfg(cli_port, cfg_file, baud=CLI_BAUD):
    cfg = pathlib.Path(cfg_file)
    if not cfg.exists():
        raise FileNotFoundError(cfg)

    with open_cli(cli_port, baud) as ser:
        # make sure nothing is running
        for pre in ("sensorStop", "flushCfg"):
            ser.write(pre.encode("ascii") + b"\n")
            response = read_until_prompt(ser, 3.0).decode(errors="ignore")
            cleaned = clean_response(response)
            if cleaned:
                sys.stdout.write(f"[{pre}] {cleaned}\n")

        for i, raw in enumerate(cfg.read_text(encoding="utf-8", errors="ignore").splitlines(
), 1):
            line = normalize_line(raw.strip())
            if not line or line.startswith(COMMENT_PREFIXES):
                continue
            # send one line, wait for prompt/Done/Error
            ser.write(line.encode("ascii") + b"\n")  # NOTE: '\n' only
            ser.flush()
            time.sleep(0.02)  # tiny pacing so the target's UART buffer never starves
            resp = read_until_prompt(ser, timeout=6.0)

            sys.stdout.write(f"[{i}] {line}\n")
            cleaned_resp = clean_response(resp.decode(errors="ignore"))
            if cleaned_resp:
                sys.stdout.write(f"    → {cleaned_resp}\n")

            if b"not recognized as a CLI command" in resp or b"Error" in resp:
                # bail early so you can see the exact failing point
                break

        # If your .cfg does not include sensorStart, uncomment:
        # ser.write(b"sensorStart\n")
        # sys.stdout.write(read_until_prompt(ser, 5.0).decode(errors="ignore"))

        # After config is sent, continuously read and display debug messages
        print("\n" + "="*60)
        print("CONFIG SENT - Now monitoring debug output...")
        print("Press Ctrl+C to stop monitoring")
        print("="*60)

        try:
            while True:
                # Read any available data
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting)
                    if data:
                        output = data.decode(errors='ignore')
                        # Filter and highlight our debug messages
                        for line in output.split('\n'):
                            if any(keyword in line for keyword in [
                                'ENET_STREAM:', 'TCP Client:', 'Object', 'CFAR',
                                'streamEnable', 'numObjOut', 'Semaphore', 'Connection'
                            ]):
                                print(f"[DEBUG] {line}")
                            elif line.strip():  # Print other non-empty lines
                                print(f"[INFO]  {line}")
                        sys.stdout.flush()
                else:
                    time.sleep(0.1)  # Small delay to prevent busy waiting

        except KeyboardInterrupt:
            print("\n" + "="*60)
            print("Monitoring stopped by user")
            print("="*60)

def monitor_debug_output(cli_port, baud=CLI_BAUD, duration=None):
    """
    Standalone function to just monitor debug output from radar
    """
    print(f"Monitoring debug output on {cli_port}...")
    print("Press Ctrl+C to stop")
    print("="*60)

    with open_cli(cli_port, baud) as ser:
        start_time = time.time()
        try:
            while True:
                if duration and (time.time() - start_time) > duration:
                    break

                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting)
                    if data:
                        output = data.decode(errors='ignore')
                        for line in output.split('\n'):
                            if any(keyword in line for keyword in [
                                'ENET_STREAM:', 'TCP Client:', 'Object', 'CFAR',
                                'streamEnable', 'numObjOut', 'Semaphore', 'Connection'
                            ]):
                                print(f"[DEBUG] {line}")
                            elif line.strip():
                                print(f"[INFO]  {line}")
                        sys.stdout.flush()
                else:
                    time.sleep(0.1)

        except KeyboardInterrupt:
            print(f"\nMonitoring stopped after {time.time() - start_time:.1f} seconds")

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "monitor":
        # Just monitor debug output without sending config
        monitor_debug_output(CLI_PORT)
    else:
        # Send config and then monitor
        send_cfg(CLI_PORT, CFG_PATH)
