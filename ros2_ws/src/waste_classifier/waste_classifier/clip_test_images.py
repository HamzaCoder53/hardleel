#!/usr/bin/env python3
"""Standalone CLI (no ROS): classify image files with the CLIP classifier and print label/conf/time per image.

Usage: python3 -m waste_classifier.clip_test_images <folder_or_files...> [--config clip_classifier.yaml]
           [--expect-from-filename]    # file 'plastic_03.jpg' expects label 'plastic' -> accuracy + confusion table
Each image is voted on its own (single frame) with the config's label_map and min_conf.
"""
import argparse
import glob
import os
import time
from collections import Counter

from waste_classifier import clip_core as core

EXTS = ('.jpg', '.jpeg', '.png', '.bmp', '.webp')


def collect(paths):
    files = []
    for p in paths:
        if os.path.isdir(p):
            files += sorted(f for f in glob.glob(os.path.join(p, '*')) if f.lower().endswith(EXTS))
        else:
            files.append(p)
    return files


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('paths', nargs='+')
    ap.add_argument('--config', default=None)
    ap.add_argument('--expect-from-filename', action='store_true')
    a = ap.parse_args(argv)
    cfg = core.load_config(a.config or core.default_config_path())
    from PIL import Image
    clf = core.ClipClassifier(cfg)
    clf.classify_image(Image.new('RGB', (320, 240)))  # warm-up
    print('model load %.1f s' % clf.load_s)
    min_conf = float(cfg.get('min_conf', 0.45))
    conf_tab, times, ok, n_exp = Counter(), [], 0, 0
    for f in collect(a.paths):
        img = Image.open(f)
        t0 = time.time()
        scores = clf.classify_image(img)
        dt = time.time() - t0
        times.append(dt)
        label, conf, _ = core.vote([scores], cfg['label_map'], min_conf)
        top = max(scores, key=scores.get)
        line = '%-28s %-8s %.2f  (top group %s %.2f)  %.0f ms' % (os.path.basename(f), label, conf, top, scores[top], dt * 1e3)
        if a.expect_from_filename:
            exp = os.path.basename(f).split('_')[0].lower()
            n_exp += 1
            ok += exp == label
            conf_tab[(exp, label)] += 1
            line += '  ' + ('OK' if exp == label else 'WRONG (expected %s)' % exp)
        print(line)
    if times:
        print('mean latency %.0f ms over %d images' % (sum(times) / len(times) * 1e3, len(times)))
    if n_exp:
        print('accuracy %d/%d = %.0f%%' % (ok, n_exp, 100.0 * ok / n_exp))
        exps = sorted({e for e, _ in conf_tab})
        preds = sorted({p for _, p in conf_tab})
        print('confusion (rows expected, cols predicted)')
        print('%-10s' % '' + ''.join('%-9s' % p for p in preds))
        for e in exps:
            print('%-10s' % e + ''.join('%-9d' % conf_tab[(e, p)] for p in preds))


if __name__ == '__main__':
    main()
