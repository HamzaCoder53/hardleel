import cv2
import numpy as np

try:
    from ai_edge_litert.interpreter import Interpreter
except ImportError:
    from tflite_runtime.interpreter import Interpreter


class WasteModel:
    """norm: 'raw' = 0-255 (model rescales itself), 'unit' = 0-1, 'signed' = -1..1"""

    def __init__(self, model_path, labels_path, norm='raw', num_threads=4):
        self.interpreter = Interpreter(model_path=model_path, num_threads=num_threads)
        self.interpreter.allocate_tensors()
        self.inp = self.interpreter.get_input_details()[0]
        self.out = self.interpreter.get_output_details()[0]
        self.h, self.w = int(self.inp['shape'][1]), int(self.inp['shape'][2])
        self.norm = norm
        with open(labels_path) as f:
            self.labels = [line.strip() for line in f if line.strip()]

    def predict(self, bgr):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, (self.w, self.h))
        x = rgb.astype(np.float32)
        if self.norm == 'unit':
            x = x / 255.0
        elif self.norm == 'signed':
            x = x / 127.5 - 1.0
        x = np.expand_dims(x, 0)
        self.interpreter.set_tensor(self.inp['index'], x)
        self.interpreter.invoke()
        probs = self.interpreter.get_tensor(self.out['index'])[0]
        i = int(np.argmax(probs))
        return self.labels[i], float(probs[i]), probs
