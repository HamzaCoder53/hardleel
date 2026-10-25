#!/usr/bin/env python3
"""Download the CLIP weights into cache_dir so the node can run offline (needs internet once).

Usage: python3 fetch_clip_weights.py [--model ViT-B-32] [--pretrained laion2b_s34b_b79k] [--cache-dir ~/.cache/clip_weights]
Prints the cache path and size. Standalone: only needs open_clip_torch + torch.
"""
import argparse
import os


def dir_size_mb(path):
    return sum(os.path.getsize(os.path.join(r, f)) for r, _, fs in os.walk(path) for f in fs
               if not os.path.islink(os.path.join(r, f))) / 1e6  # symlinks point into blobs/


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='ViT-B-32')
    ap.add_argument('--pretrained', default='laion2b_s34b_b79k')
    ap.add_argument('--cache-dir', default='~/.cache/clip_weights')
    a = ap.parse_args()
    cache = os.path.expanduser(a.cache_dir)
    os.makedirs(cache, exist_ok=True)
    os.environ.pop('HF_HUB_OFFLINE', None)  # must be online here
    import open_clip
    open_clip.create_model_and_transforms(a.model, pretrained=a.pretrained, cache_dir=cache, device='cpu')
    open_clip.get_tokenizer(a.model)
    print('cache: %s  size: %.0f MB' % (cache, dir_size_mb(cache)))


if __name__ == '__main__':
    main()
