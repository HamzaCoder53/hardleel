import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('192.168.137.239', username='hardleel', password='hardleel', timeout=5)

for f in ['/home/hardleel/maps/my_map.yaml', '/home/hardleel/maps/track_20261015_1232.yaml', '/home/hardleel/maps/track_20261015_1232_autosave.yaml']:
    print('=== ' + f + ' ===')
    stdin, stdout, stderr = ssh.exec_command('cat ' + f)
    print(stdout.read().decode())

print('=== NAV LOG TAIL ===')
stdin, stdout, stderr = ssh.exec_command('tail -n 25 /home/hardleel/maps/nav.log')
print(stdout.read().decode())

ssh.close()
