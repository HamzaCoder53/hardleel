#!/usr/bin/env python3
"""Zero-shot CLIP waste classifier core (pure Python, no ROS). Runs on the laptop, CPU only.

    ClipClassifier(cfg).classify_image(img) -> {group: prob}   softmax over ALL prompts, summed per prompt group
    vote(score_dicts, label_map, min_conf)  -> (label, conf, per_label)   mean over frames, argmax, threshold

cfg is the nested `ros__parameters` dict of config/clip_classifier.yaml (load_config()):
    model, pretrained, cache_dir, offline, num_threads, rotate_deg (0/90/180/270), crop_frac (0..1 centre crop),
    prompts {group: [text, ...]}, label_map {group: final label}, min_conf, colours {label: colour}
Offline mode sets HF_HUB_OFFLINE=1, so the weights must be cached first (scripts/fetch_clip_weights.py).
"""
import os
import time

UNKNOWN = 'unknown'


def load_config(path):
    """Read a ROS params yaml and return the nested ros__parameters dict of its first node."""
    import yaml
    with open(os.path.expanduser(path)) as f:
        data = yaml.safe_load(f)
    node = next(iter(data.values()))
    return node['ros__parameters']


def default_config_path():
    """Config path from the installed share dir, else the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('waste_classifier'), 'config', 'clip_classifier.yaml')
        if os.path.exists(p):
            return p
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'clip_classifier.yaml')


def prepare_image(img, rotate_deg=0, crop_frac=1.0):
    """numpy (RGB) or PIL image -> PIL RGB, rotated by a multiple of 90 deg (counter-clockwise) and centre-cropped."""
    from PIL import Image
    if not isinstance(img, Image.Image):
        img = Image.fromarray(img)
    img = img.convert('RGB')
    rot = int(rotate_deg) % 360
    if rot not in (0, 90, 180, 270):
        raise ValueError('rotate_deg must be 0/90/180/270, got %s' % rotate_deg)
    if rot:
        img = img.rotate(rot, expand=True)
    if 0.0 < crop_frac < 1.0:
        w, h = img.size
        cw, ch = max(1, int(w * crop_frac)), max(1, int(h * crop_frac))
        x0, y0 = (w - cw) // 2, (h - ch) // 2
        img = img.crop((x0, y0, x0 + cw, y0 + ch))
    return img


def map_to_labels(scores, label_map):
    """{group: prob} -> {label: prob}; groups missing from label_map count as unknown."""
    out = {}
    for group, p in scores.items():
        lab = label_map.get(group, UNKNOWN)
        out[lab] = out.get(lab, 0.0) + float(p)
    return out


def vote(score_dicts, label_map, min_conf):
    """Average per-label probabilities over frames; argmax. Below min_conf or label unknown -> 'unknown'.
    Returns (label, conf, per_label_mean). Empty input -> ('unknown', 0.0, {})."""
    if not score_dicts:
        return UNKNOWN, 0.0, {}
    total = {}
    for s in score_dicts:
        for lab, p in map_to_labels(s, label_map).items():
            total[lab] = total.get(lab, 0.0) + p
    per_label = {lab: p / len(score_dicts) for lab, p in total.items()}
    best = max(per_label, key=per_label.get)
    conf = per_label[best]
    if best == UNKNOWN or conf < min_conf:
        return UNKNOWN, conf, per_label
    return best, conf, per_label


class ClipClassifier:
    def __init__(self, cfg, log=print):
        self.cfg = cfg
        self.rotate_deg = int(cfg.get('rotate_deg', 0))
        self.crop_frac = float(cfg.get('crop_frac', 1.0))
        self.groups = list(cfg['prompts'].keys())
        self.texts, self.text_group = [], []
        for g in self.groups:
            for t in cfg['prompts'][g]:
                self.texts.append(t)
                self.text_group.append(g)
        t0 = time.time()
        self._load(log)
        self.load_s = time.time() - t0

    def _load(self, log):
        cache_dir = os.path.expanduser(str(self.cfg.get('cache_dir', '~/.cache/clip_weights')))
        model_name = self.cfg.get('model', 'ViT-B-32')
        pretrained = self.cfg.get('pretrained', 'laion2b_s34b_b79k')
        if self.cfg.get('offline', True):
            os.environ['HF_HUB_OFFLINE'] = '1'
        try:
            import torch
            import open_clip
        except ImportError as e:
            raise RuntimeError('open_clip/torch missing (%s): pip3 install --user open_clip_torch torch '
                               'torchvision --index-url https://download.pytorch.org/whl/cpu' % e)
        torch.set_num_threads(int(self.cfg.get('num_threads', 4)))
        self.torch = torch
        try:
            model, _, preprocess = open_clip.create_model_and_transforms(
                model_name, pretrained=pretrained, cache_dir=cache_dir, device='cpu')
        except Exception as e:
            raise RuntimeError('cannot load CLIP %s/%s from %s (offline=%s): %s. Run '
                               'scripts/fetch_clip_weights.py while online first.'
                               % (model_name, pretrained, cache_dir, self.cfg.get('offline', True), e))
        model.eval()
        self.model, self.preprocess = model, preprocess
        tok = open_clip.get_tokenizer(model_name)
        with torch.no_grad():
            tf = model.encode_text(tok(self.texts))
            self.text_feat = tf / tf.norm(dim=-1, keepdim=True)
            self.logit_scale = float(model.logit_scale.exp())
        log('CLIP %s/%s loaded: %d prompts in %d groups' % (model_name, pretrained, len(self.texts), len(self.groups)))

    def classify_image(self, img):
        """One image -> {group: prob}; probs sum to 1 over all groups."""
        torch = self.torch
        pil = prepare_image(img, self.rotate_deg, self.crop_frac)
        x = self.preprocess(pil).unsqueeze(0)
        with torch.no_grad():
            f = self.model.encode_image(x)
            f = f / f.norm(dim=-1, keepdim=True)
            probs = (self.logit_scale * f @ self.text_feat.T).softmax(dim=-1)[0].tolist()
        out = {g: 0.0 for g in self.groups}
        for g, p in zip(self.text_group, probs):
            out[g] += p
        return out
