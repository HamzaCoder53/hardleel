"""Unit tests for clip_core vote/label mapping/rotation (no model download needed)."""
import os

import numpy as np
import pytest

from waste_classifier import clip_core as core

LM = {'plastic': 'plastic', 'metal': 'metal', 'organic': 'organic', 'fabric': 'organic',
      'paper': 'organic', 'glass': 'unknown', 'empty': 'unknown'}


def s(**kw):
    d = {g: 0.0 for g in LM}
    d.update(kw)
    return d


def test_vote_basic():
    label, conf, per = core.vote([s(plastic=0.8, metal=0.1, empty=0.1)] * 3, LM, 0.45)
    assert label == 'plastic' and conf == pytest.approx(0.8)
    assert per['unknown'] == pytest.approx(0.1)


def test_groups_sum_into_label():
    label, conf, per = core.vote([s(organic=0.3, fabric=0.2, paper=0.1, plastic=0.4)], LM, 0.45)
    assert label == 'organic' and conf == pytest.approx(0.6)


def test_below_min_conf_is_unknown():
    label, conf, _ = core.vote([s(plastic=0.4, metal=0.3, organic=0.3)], LM, 0.45)
    assert label == 'unknown' and conf == pytest.approx(0.4)


def test_unknown_winner():
    label, _, _ = core.vote([s(empty=0.7, glass=0.1, plastic=0.2)], LM, 0.45)
    assert label == 'unknown'


def test_average_over_frames():
    frames = [s(metal=0.9, plastic=0.1), s(metal=0.2, plastic=0.8), s(metal=0.7, plastic=0.3)]
    label, conf, per = core.vote(frames, LM, 0.45)
    assert label == 'metal' and conf == pytest.approx(0.6)


def test_empty_input():
    assert core.vote([], LM, 0.45) == ('unknown', 0.0, {})


def test_missing_group_is_unknown():
    assert core.map_to_labels({'mystery': 1.0}, LM) == {'unknown': 1.0}


def test_rotation_and_crop():
    img = np.zeros((40, 80, 3), np.uint8)  # h=40, w=80
    assert core.prepare_image(img, 0).size == (80, 40)
    assert core.prepare_image(img, 90).size == (40, 80)
    assert core.prepare_image(img, 180).size == (80, 40)
    assert core.prepare_image(img, 270).size == (40, 80)
    assert core.prepare_image(img, 0, 0.5).size == (40, 20)
    with pytest.raises(ValueError):
        core.prepare_image(img, 45)


def test_rotation_moves_pixels():
    img = np.zeros((4, 8, 3), np.uint8)
    img[0, 0] = 255  # top-left; CCW 90 -> bottom-left
    out = np.array(core.prepare_image(img, 90))
    assert out[-1, 0].tolist() == [255, 255, 255]


def test_config_file_consistent():
    cfg = core.load_config(os.path.join(os.path.dirname(__file__), '..', 'config', 'clip_classifier.yaml'))
    assert set(cfg['prompts']) == set(cfg['label_map'])
    assert cfg['label_map']['fabric'] == 'organic' and cfg['colours']['plastic'] == 'blue'


def test_model_smoke():
    pytest.importorskip('open_clip')
    cfg = core.load_config(os.path.join(os.path.dirname(__file__), '..', 'config', 'clip_classifier.yaml'))
    try:
        clf = core.ClipClassifier(cfg, log=lambda m: None)
    except RuntimeError:
        pytest.skip('weights not cached')
    out = clf.classify_image(np.zeros((100, 100, 3), np.uint8))
    assert sum(out.values()) == pytest.approx(1.0, abs=1e-3)
