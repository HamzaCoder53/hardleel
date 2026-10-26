import argparse
import glob
import os

import cv2
from ament_index_python.packages import get_package_share_directory

from waste_classifier.classifier import WasteModel


def main():
    share = get_package_share_directory('waste_classifier')
    p = argparse.ArgumentParser()
    p.add_argument('folder', help='folder with .jpg/.jpeg/.png test photos')
    p.add_argument('--model', default=f'{share}/models/waste_model.tflite')
    p.add_argument('--labels', default=f'{share}/models/labels.txt')
    p.add_argument('--norm', default='raw', choices=['raw', 'unit', 'signed'])
    args = p.parse_args()

    model = WasteModel(args.model, args.labels, norm=args.norm)
    folder = os.path.expanduser(args.folder)
    files = sorted(
        f for ext in ('*.jpg', '*.jpeg', '*.png', '*.JPG', '*.JPEG', '*.PNG')
        for f in glob.glob(os.path.join(folder, ext)))
    if not files:
        print(f'No images found in {folder}')
        return
    print(f'norm={args.norm}  labels={model.labels}')
    for f in files:
        img = cv2.imread(f)
        if img is None:
            print(f'{os.path.basename(f)}: could not read')
            continue
        label, conf, probs = model.predict(img)
        detail = '  '.join(f'{l}={p:.2f}' for l, p in zip(model.labels, probs))
        print(f'{os.path.basename(f):30s} -> {label} ({conf:.2f})   [{detail}]')


if __name__ == '__main__':
    main()
