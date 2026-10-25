# COLD START, laptop side - the ONE command after a laptop reboot and/or a Pi power-cycle.
# Run in an ADMIN PowerShell from the repo folder (Documents\hardleel-pi-repo\.claude\worktrees\mission-m1):
#     powershell -ExecutionPolicy Bypass -File scripts\laptop\up.ps1            # phase 1 (mapping) windows
#     powershell -ExecutionPolicy Bypass -File scripts\laptop\up.ps1 -NoPi       # Pi already running
#     ... -NoPi -CheckOnly    steps 1-5 only, no windows (tested 25 Oct 16:08: Pi found, proxy, topics OK)
# Steps: 1 hotspot check  2 find the Pi  3 port proxy + firewall (re)pointed at the Pi  4 start the Pi drivers
#        (robot must stand STILL for 5 s after this: IMU calibration)  5 WSL check  6 open the mapping + teleop
#        windows (Jazzy distro). The Pi asks for its password unless an ssh key is set up (RUNBOOK).
param([switch]$NoPi, [switch]$CheckOnly, [string]$Distro = "Ubuntu-24.04", [string]$PiUser = "hardleel")
$ErrorActionPreference = "Stop"
function Say($t) { Write-Host "== $t" -ForegroundColor Cyan }

# 1. hotspot
Say "1/6 hotspot"
$hs = Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue | Where-Object { $_.IPAddress -like "192.168.137.*" }
if (-not $hs) { Write-Host "Mobile hotspot is OFF: Settings > Network > Mobile hotspot > On, then rerun." -ForegroundColor Red; exit 1 }

# 2. Pi address: hostname first, then the known MAC on the hotspot, then the usual IP
Say "2/6 finding the Pi"
$PiIp = $null
try { $PiIp = (Resolve-DnsName hardleel.local -Type A -ErrorAction Stop | Select-Object -First 1).IPAddress } catch {}
if (-not $PiIp) {
    $n = Get-NetNeighbor -AddressFamily IPv4 -ErrorAction SilentlyContinue | Where-Object { $_.LinkLayerAddress -eq "2C-CF-67-D8-3D-65" }
    if ($n) { $PiIp = $n[0].IPAddress }
}
if (-not $PiIp) { $PiIp = "192.168.137.8" }
for ($i = 0; $i -lt 30 -and -not (Test-Connection $PiIp -Count 1 -Quiet); $i++) { Write-Host "  waiting for the Pi at $PiIp ..."; Start-Sleep 2 }
if (-not (Test-Connection $PiIp -Count 1 -Quiet)) { Write-Host "Pi not reachable at $PiIp (booting? on the hotspot?)" -ForegroundColor Red; exit 1 }
Write-Host "  Pi = $PiIp"

# 3. port proxy (WSL cannot reach the hotspot subnet; zenoh goes Windows 0.0.0.0:7447 -> Pi:7447)
Say "3/6 port proxy -> $PiIp`:7447"
netsh interface portproxy delete v4tov4 listenport=7447 listenaddress=0.0.0.0 2>$null | Out-Null
netsh interface portproxy add v4tov4 listenport=7447 listenaddress=0.0.0.0 connectport=7447 connectaddress=$PiIp | Out-Null
if (-not (Get-NetFirewallRule -DisplayName "hardleel zenoh from WSL" -ErrorAction SilentlyContinue)) {
    New-NetFirewallRule -DisplayName "hardleel zenoh from WSL" -Direction Inbound -Protocol TCP -LocalPort 7447 `
        -InterfaceAlias "vEthernet (WSL (Hyper-V firewall))" -Action Allow | Out-Null
}
if ((Get-Service iphlpsvc).Status -ne "Running") { Start-Service iphlpsvc }

# 4. Pi drivers (zenoh router + lidar + pi_bringup in tmux "hw")
if (-not $NoPi) {
    Say "4/6 starting the Pi drivers - DO NOT TOUCH THE ROBOT for 5 s after this (IMU calibration)"
    ssh -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new "$PiUser@$PiIp" "bash ~/robot_hw.sh"
} else { Say "4/6 Pi: skipped (-NoPi)" }

# 5. WSL: the laptop router is started by ros_env.sh; check that the Pi topics arrive
Say "5/6 checking Pi topics from WSL ($Distro)"
wsl -d $Distro -- bash -lc "cd ~/hardleel && PI_IP=$PiIp source scripts/laptop/ros_env.sh && python3 scripts/laptop/topic_probe.py 5 /scan /imu/data /camera/image_raw/compressed 2>&1 | grep Hz"

if ($CheckOnly) { Say "check only: not opening the mapping/teleop windows"; exit 0 }
# 6. windows: A = mapping (Cartographer etc., keep it open into phase 2), B = teleop
Say "6/6 opening terminal A (laptop_mapping) and terminal B (teleop)"
Start-Process wsl.exe -ArgumentList "-d", $Distro, "--", "bash", "-lc", "cd ~/hardleel && PI_IP=$PiIp source scripts/laptop/ros_env.sh && ros2 launch robot_slam laptop_mapping.launch.py; exec bash"
Start-Sleep 3
Start-Process wsl.exe -ArgumentList "-d", $Distro, "--", "bash", "-lc", "cd ~/hardleel && PI_IP=$PiIp source scripts/laptop/ros_env.sh && scripts/laptop/teleop.sh; exec bash"
Say "UP. Phase 1: drive with terminal B. Then RUNBOOK section 3 (save_map.sh, close teleop, laptop_mission)."
