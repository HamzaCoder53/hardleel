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
# Always recreate the rule: Windows can change the WSL adapter's interface ID (seen 31 Oct 2026, after an update), and an
# old rule then silently blocks WSL -> 7447 ("NO PUBLISHER" for every Pi topic) while still looking present.
Get-NetFirewallRule -DisplayName "hardleel zenoh from WSL" -ErrorAction SilentlyContinue | Remove-NetFirewallRule
New-NetFirewallRule -DisplayName "hardleel zenoh from WSL" -Direction Inbound -Protocol TCP -LocalPort 7447 `
    -InterfaceAlias "vEthernet (WSL (Hyper-V firewall))" -Action Allow | Out-Null
if ((Get-Service iphlpsvc).Status -ne "Running") { Start-Service iphlpsvc }

# 4. Pi drivers (zenoh router + lidar + pi_bringup in tmux "hw")
#    The Pi has no RTC and the hotspot gives it no DNS: after a power-cycle its clock can be days off, which
#    breaks every sensor timestamp. So the laptop's time is sent in the SAME ssh session, right after login
#    (the Pi prints READY, we answer with the current epoch to the ms), then the drivers start.
function Start-PiWithTime($target, $cmd) {
    $psi = New-Object System.Diagnostics.ProcessStartInfo
    $psi.FileName = 'ssh'
    $psi.Arguments = "-o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new $target `"$cmd`""
    $psi.UseShellExecute = $false
    $psi.RedirectStandardInput = $true
    $psi.RedirectStandardOutput = $true
    $p = [System.Diagnostics.Process]::Start($psi)      # the password prompt still comes up in this console
    while (-not $p.StandardOutput.EndOfStream) {
        $line = $p.StandardOutput.ReadLine()
        if ($line -eq 'READY') {
            $ep = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds() / 1000.0
            # plain ASCII bytes: WriteLine() would add a BOM / CR that `date` rejects (tested 25 Oct)
            $b = [Text.Encoding]::ASCII.GetBytes($ep.ToString([Globalization.CultureInfo]::InvariantCulture) + "`n")
            $p.StandardInput.BaseStream.Write($b, 0, $b.Length)
            $p.StandardInput.BaseStream.Flush()
            $p.StandardInput.Close()
        } else { Write-Host "  pi: $line" }
    }
    $p.WaitForExit()
}
if (-not $NoPi) {
    Say "4/6 setting the Pi clock + starting the Pi drivers - DO NOT TOUCH THE ROBOT for 5 s after this (IMU calibration)"
    Start-PiWithTime "$PiUser@$PiIp" 'echo READY; read EP; EP=$(printf %s \"$EP\" | tr -cd 0-9.); sudo -n date -u -s @$EP >/dev/null && echo clock set: $(date); bash ~/robot_hw.sh'
} else { Say "4/6 Pi: skipped (-NoPi)" }

# 5. WSL: the laptop router is started by ros_env.sh; check that the Pi topics arrive
Say "5/6 checking Pi topics from WSL ($Distro)"
# fresh laptop router: after a Pi reboot the old link still lists the topics but carries no data (25 Oct)
wsl -d $Distro -- bash -lc "pkill -x rmw_zenohd; sleep 1; cd ~/hardleel && PI_IP=$PiIp source scripts/laptop/ros_env.sh && sleep 2 && python3 scripts/laptop/topic_probe.py 5 /scan /imu/data /camera/image_raw/compressed 2>&1 | grep Hz"

if ($CheckOnly) { Say "check only: not opening the mapping/teleop windows"; exit 0 }
# 6. windows: A = mapping (Cartographer etc., keep it open into phase 2), B = teleop
Say "6/6 opening terminal A (laptop_mapping) and terminal B (teleop)"
Start-Process wsl.exe -ArgumentList "-d", $Distro, "--", "bash", "-lc", "cd ~/hardleel && PI_IP=$PiIp source scripts/laptop/ros_env.sh && ros2 launch robot_slam laptop_mapping.launch.py; exec bash"
Start-Sleep 3
Start-Process wsl.exe -ArgumentList "-d", $Distro, "--", "bash", "-lc", "cd ~/hardleel && PI_IP=$PiIp source scripts/laptop/ros_env.sh && scripts/laptop/teleop.sh; exec bash"
Say "UP. Phase 1: drive with terminal B. Then RUNBOOK section 3 (save_map.sh, close teleop, laptop_mission)."
