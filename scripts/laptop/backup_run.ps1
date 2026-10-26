# Back up one run: Pi -> laptop (resumable, checksummed), + the laptop's own run folder, -> Dest (Google Drive
# folder or external drive), verified again. The Pi copy is deleted ONLY with -DeletePi and only when BOTH copies
# match the Pi's SHA-256 manifest.
#   powershell -ExecutionPolicy Bypass -File scripts\laptop\backup_run.ps1 -Run run_20261101_101500 `
#       -Dest "G:\My Drive\hardleel_runs" [-PiIp 192.168.137.231] [-DeletePi]
# Needs the ssh key login to the Pi (ssh-agent). Staging: %USERPROFILE%\hardleel_runs\<run>.
param([Parameter(Mandatory = $true)][string]$Run, [string]$Dest = "", [string]$PiIp = "", [string]$PiUser = "hardleel",
      [string]$Distro = "Ubuntu-24.04", [string]$WslUser = "hamza", [switch]$DeletePi)
$ErrorActionPreference = "Stop"
function Say($t) { Write-Host "== $t" -ForegroundColor Cyan }
if (-not $PiIp) {   # same discovery as up.ps1: the Pi's MAC on the hotspot
    $n = Get-NetNeighbor -AddressFamily IPv4 -ErrorAction SilentlyContinue | Where-Object { $_.LinkLayerAddress -eq '2C-CF-67-D8-3D-65' }
    foreach ($h in $n) { if (Test-Connection $h.IPAddress -Count 1 -Quiet) { $PiIp = $h.IPAddress } }
}
$stage = Join-Path $env:USERPROFILE "hardleel_runs\$Run"
New-Item -ItemType Directory -Force $stage | Out-Null

function Hashes($dir) {   # relative path (forward slashes) -> sha256 (lower case)
    $h = @{}
    if (-not (Test-Path $dir)) { return $h }
    Get-ChildItem $dir -Recurse -File | ForEach-Object {
        $rel = $_.FullName.Substring($dir.Length).TrimStart('\').Replace('\', '/')
        $h[$rel] = (Get-FileHash $_.FullName -Algorithm SHA256).Hash.ToLower()
    }
    return $h
}

$piManifest = @{}
if ($PiIp) {
    Say "1/4 Pi $PiIp -> $stage (sftp, resumable)"
    $exists = ssh -o BatchMode=yes -o ConnectTimeout=10 "$PiUser@$PiIp" "test -d ~/runs/$Run && echo yes"
    if ($exists -eq "yes") {
        $batch = Join-Path $env:TEMP "sftp_$Run.txt"
        Set-Content -Encoding ascii $batch "lcd `"$($stage.Replace('\','/'))/..`"`nget -a -R runs/$Run"
        sftp -b $batch -o BatchMode=yes "$PiUser@$PiIp" | Out-Null
        foreach ($line in (ssh -o BatchMode=yes "$PiUser@$PiIp" "cd ~/runs/$Run && find . -type f -exec sha256sum {} +")) {
            $p = $line -split '\s+', 2
            if ($p.Count -eq 2) { $piManifest[$p[1].TrimStart('.', '/')] = $p[0] }
        }
        $local = Hashes $stage
        $bad = @($piManifest.Keys | Where-Object { $local[$_] -ne $piManifest[$_] })
        if ($bad.Count) { Write-Host "Pi copy MISMATCH ($($bad.Count) files): $($bad -join ', ')" -ForegroundColor Red; exit 1 }
        Write-Host "  Pi copy verified: $($piManifest.Count) files"
    } else { Write-Host "  no ~/runs/$Run on the Pi (laptop-only run)" }
} else { Write-Host "  Pi not found: skipping the Pi side" -ForegroundColor Yellow }

Say "2/4 laptop (WSL) run folder -> $stage"
$wsl = "\\wsl$\$Distro\home\$WslUser\runs\$Run"
if (Test-Path $wsl) { robocopy $wsl $stage /E /Z /R:2 /W:2 /NFL /NDL /NJH /NJS | Out-Null } else { Write-Host "  none at $wsl" }

$stageHash = Hashes $stage
Write-Host "  staging: $($stageHash.Count) files, $([math]::Round(((Get-ChildItem $stage -Recurse -File | Measure-Object Length -Sum).Sum)/1MB,1)) MB"
$destOk = $false
if ($Dest) {
    Say "3/4 $stage -> $Dest\$Run"
    $target = Join-Path $Dest $Run
    robocopy $stage $target /E /Z /R:3 /W:5 /NFL /NDL /NJH /NJS | Out-Null
    $dh = Hashes $target
    $bad = @($stageHash.Keys | Where-Object { $dh[$_] -ne $stageHash[$_] })
    if ($bad.Count) { Write-Host "DEST MISMATCH ($($bad.Count) files)" -ForegroundColor Red; exit 1 }
    $destOk = $true
    Write-Host "  destination verified: $($dh.Count) files"
} else { Write-Host "  no -Dest given: only the laptop copy exists" -ForegroundColor Yellow }

Say "4/4 Pi copy"
if ($DeletePi -and $PiIp -and $piManifest.Count -and $destOk) {
    ssh -o BatchMode=yes "$PiUser@$PiIp" "rm -rf ~/runs/$Run && echo deleted ~/runs/$Run on the Pi"
} else { Write-Host "  kept on the Pi (delete needs -DeletePi AND verified laptop + destination copies)" }
