# Standalone RPLIDAR A1 check: .NET SerialPort + the Slamtec serial protocol written out by hand.
# Shares no code with sllidar_ros2 / the Slamtec SDK. usage: lidar_check.ps1 -Port COM21 [-Seconds 6]
param([string]$Port = 'COM21', [int]$Seconds = 6, [string]$Png = "$PSScriptRoot\lidar_scan.png")
$ErrorActionPreference = 'Stop'
$sp = New-Object System.IO.Ports.SerialPort $Port, 115200, 'None', 8, 'One'
$sp.ReadTimeout = 300; $sp.DtrEnable = $false; $sp.RtsEnable = $false   # DTR low = A1 motor on
$sp.Open()
function Send([byte]$cmd) { $sp.Write([byte[]](0xA5, $cmd), 0, 2) }
function ReadN([int]$n, [int]$ms) {
    $buf = New-Object byte[] $n; $got = 0; $sw = [Diagnostics.Stopwatch]::StartNew()
    while ($got -lt $n -and $sw.ElapsedMilliseconds -lt $ms) {
        if ($sp.BytesToRead -gt 0) { $got += $sp.Read($buf, $got, [Math]::Min($n - $got, $sp.BytesToRead)) } else { Start-Sleep -Milliseconds 5 }
    }
    if ($got -lt $n) { return $null } ; return $buf
}
function Ask([byte]$cmd, [int]$len, [int]$tries = 3) {
    for ($t = 1; $t -le $tries; $t++) {
        $sp.DiscardInBuffer(); Send $cmd
        $r = ReadN (7 + $len) 1500
        if ($r -and $r[0] -eq 0xA5 -and $r[1] -eq 0x5A) { return @{ data = $r[7..(6 + $len)]; tries = $t } }
        Start-Sleep -Milliseconds 300
    }
    return $null
}
"Port $Port open, motor on (DTR low). Waiting 2 s for spin-up..."
Send 0x25; Start-Sleep -Seconds 2; $sp.DiscardInBuffer()

$info = Ask 0x50 20
if ($info) {
    $d = $info.data; $sn = ($d[4..19] | ForEach-Object { $_.ToString('X2') }) -join ''
    "GET_INFO   : OK (try $($info.tries)) model=0x$($d[0].ToString('X2')) firmware=$($d[2]).$($d[1].ToString('00')) hardware=$($d[3]) S/N=$sn"
} else { "GET_INFO   : NO REPLY after 3 tries" }
$h = Ask 0x52 3
if ($h) { $st = @('OK', 'Warning', 'Error')[$h.data[0]]; "GET_HEALTH : $st (status=$($h.data[0]), error code=$($h.data[1] + 256 * $h.data[2]), try $($h.tries))" }
else { "GET_HEALTH : NO REPLY after 3 tries" }
$r = Ask 0x59 4
if ($r) { "SAMPLERATE : standard $($r.data[0] + 256 * $r.data[1]) us/sample, express $($r.data[2] + 256 * $r.data[3]) us/sample" } else { "SAMPLERATE : NO REPLY" }

# Plain SCAN (0xA5 0x20): descriptor A5 5A 05 00 00 40 81, then 5-byte measurement nodes
$sp.DiscardInBuffer(); Send 0x20
$desc = ReadN 7 2000
if (-not $desc -or $desc[0] -ne 0xA5 -or $desc[1] -ne 0x5A) {
    "SCAN       : NO scan descriptor (lidar did not start scanning)"
    Send 0x25; $sp.DtrEnable = $true; $sp.Close(); exit 2
}
"SCAN       : started, collecting $Seconds s..."
$stream = New-Object System.Collections.Generic.List[byte]
$sw = [Diagnostics.Stopwatch]::StartNew()
$chunk = New-Object byte[] 4096
while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
    $n = $sp.BytesToRead
    if ($n -gt 0) { $k = $sp.Read($chunk, 0, [Math]::Min($n, 4096)); $stream.AddRange([byte[]]$chunk[0..($k - 1)]) } else { Start-Sleep -Milliseconds 10 }
}
Send 0x25; Start-Sleep -Milliseconds 200; $sp.DtrEnable = $true; $sp.Close()   # stop scan + motor

$b = $stream.ToArray(); $i = 0; $bad = 0; $revs = New-Object System.Collections.Generic.List[object]; $cur = $null
while ($i + 5 -le $b.Length) {
    $s = $b[$i] -band 1; $ns = ($b[$i] -shr 1) -band 1; $c = $b[$i + 1] -band 1
    if ($s -eq $ns -or $c -ne 1) { $i++; $bad++; continue }          # resync on a malformed node
    $q = $b[$i] -shr 2
    $ang = ((($b[$i + 1] -shr 1) -bor ($b[$i + 2] -shl 7))) / 64.0
    $dist = ($b[$i + 3] -bor ($b[$i + 4] -shl 8)) / 4.0           # mm
    if ($s -eq 1) { if ($cur) { $revs.Add($cur) }; $cur = New-Object System.Collections.Generic.List[object] }
    if ($cur) { $cur.Add(@($ang, $dist, $q)) }
    $i += 5
}
"bytes received: $($b.Length) ($([int]($b.Length / $Seconds)) B/s), malformed bytes skipped: $bad"
if ($revs.Count -lt 2) { "RESULT     : fewer than 2 full revolutions decoded - scanning is NOT working"; exit 3 }
$pts = $revs | ForEach-Object { $_.Count }; $valid = $revs | ForEach-Object { ($_ | Where-Object { $_[1] -gt 0 }).Count }
$hz = $revs.Count / $Seconds
"revolutions: $($revs.Count) in $Seconds s = $([Math]::Round($hz, 2)) Hz"
"points/rev : min $(($pts | Measure-Object -Minimum).Minimum) median $(($pts | Sort-Object)[[int]($pts.Count / 2)]) max $(($pts | Measure-Object -Maximum).Maximum)"
"valid/rev  : median $(($valid | Sort-Object)[[int]($valid.Count / 2)]) (distance > 0)"
$last = $revs[$revs.Count - 1]
$sec = 0..11 | ForEach-Object { $a0 = $_ * 30; $in = $last | Where-Object { $_[0] -ge $a0 -and $_[0] -lt $a0 + 30 }; $ok = ($in | Where-Object { $_[1] -gt 0 }); "{0,3}-{1,3}: {2,3}/{3,-3}" -f $a0, ($a0 + 30), $ok.Count, $in.Count }
"valid per 30 deg sector (last rev): " + ($sec -join ' | ')
$okd = $last | Where-Object { $_[1] -gt 0 } | ForEach-Object { $_[1] }
if ($okd) { "range mm   : min $([int]($okd | Measure-Object -Minimum).Minimum) max $([int]($okd | Measure-Object -Maximum).Maximum)" }

# picture of the last revolution (top view, lidar at the centre, 0 deg = right, 1 m grid)
Add-Type -AssemblyName System.Drawing
$W = 800; $bmp = New-Object System.Drawing.Bitmap $W, $W; $g = [System.Drawing.Graphics]::FromImage($bmp)
$g.Clear([System.Drawing.Color]::FromArgb(30, 30, 36)); $scale = $W / 2 / 4000.0   # 4 m to the edge
$pen = New-Object System.Drawing.Pen ([System.Drawing.Color]::FromArgb(70, 70, 80))
for ($m = 1; $m -le 4; $m++) { $r = $m * 1000 * $scale; $g.DrawEllipse($pen, $W / 2 - $r, $W / 2 - $r, 2 * $r, 2 * $r) }
$br = New-Object System.Drawing.SolidBrush ([System.Drawing.Color]::FromArgb(255, 60, 200))
foreach ($p in $revs[($revs.Count - 3)..($revs.Count - 1)]) { foreach ($q in $p) { if ($q[1] -gt 0) {
    $x = $W / 2 + [Math]::Cos($q[0] * [Math]::PI / 180) * $q[1] * $scale; $y = $W / 2 - [Math]::Sin($q[0] * [Math]::PI / 180) * $q[1] * $scale
    $g.FillRectangle($br, [float]$x - 1.5, [float]$y - 1.5, 3, 3) } } }
$g.FillEllipse([System.Drawing.Brushes]::Lime, $W / 2 - 5, $W / 2 - 5, 10, 10)
$g.DrawString("RPLIDAR on $Port - last 3 revolutions, rings = 1 m", (New-Object System.Drawing.Font 'Segoe UI', 12), [System.Drawing.Brushes]::White, 10, 10)
$bmp.Save($Png); "picture    : $Png"
"RESULT     : scanning WORKS"
