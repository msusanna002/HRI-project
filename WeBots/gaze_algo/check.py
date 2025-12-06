import subprocess, re

def mac_screen_resolution():
    cmd = "system_profiler SPDisplaysDataType"
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    m = re.search(r"Resolution: (\d+)\s*x\s*(\d+)", result.stdout)
    if m:
        return int(m.group(1)), int(m.group(2))
    return None

print(mac_screen_resolution())
