# Run ONCE in an ADMIN PowerShell on the laptop (persists across reboots).
# WSL cannot reach the Windows mobile-hotspot subnet (192.168.137.x), so the laptop's zenoh router connects
# to this Windows port proxy, which forwards TCP 7447 to the Pi's rmw_zenohd.
param([string]$PiIp = "192.168.137.8")
netsh interface portproxy delete v4tov4 listenport=7447 listenaddress=0.0.0.0 2>$null | Out-Null
netsh interface portproxy add v4tov4 listenport=7447 listenaddress=0.0.0.0 connectport=7447 connectaddress=$PiIp
Get-NetFirewallRule -DisplayName "hardleel zenoh from WSL" -ErrorAction SilentlyContinue | Remove-NetFirewallRule
New-NetFirewallRule -DisplayName "hardleel zenoh from WSL" -Direction Inbound -Protocol TCP -LocalPort 7447 `
    -InterfaceAlias "vEthernet (WSL (Hyper-V firewall))" -Action Allow | Out-Null
netsh interface portproxy show v4tov4
