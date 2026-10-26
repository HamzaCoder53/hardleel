import os
import sys

path = os.path.expanduser('~/map_track.sh')
s = open(path).read()
if 'SNAP=' in s:
    print('already patched')
    sys.exit(0)


def rep(old, new):
    global s
    assert s.count(old) == 1, f'anchor found {s.count(old)}x: {old!r}'
    s = s.replace(old, new)


rep('  while sleep $EVERY; do\n',
    '  N=0; SNAP=$MAPS/${NAME}_snapshots; mkdir -p $SNAP   # numbered copy of every autosave, never overwritten\n'
    '  while sleep $EVERY; do\n')
rep('      mv -f $TMP/${NAME}_autosave.* $MAPS/\n',
    '      if [ "$(df --output=avail -m $MAPS | tail -1)" -gt 2000 ]; then      # keep 2 GB free on the SD card\n'
    '        N=$((N+1)); ID=$(printf "%04d_%s" $N $(date +%H%M%S))\n'
    '        ln $TMP/${NAME}_autosave.pgm $SNAP/map_$ID.pgm\n'
    '        sed "s|^image: .*|image: map_$ID.pgm|" $TMP/${NAME}_autosave.yaml > $SNAP/map_$ID.yaml\n'
    '        [ -s $TMP/${NAME}_autosave.pbstream ] && ln $TMP/${NAME}_autosave.pbstream $SNAP/map_$ID.pbstream\n'
    '      fi\n'
    '      mv -f $TMP/${NAME}_autosave.* $MAPS/\n')
open(path, 'w').write(s)
print('patched', path)
