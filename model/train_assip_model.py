# # Export YOLO11n → Grove Vision AI V2
# Re-export existing best.pt at 192x192, INT8 TFLite, Vela compiled

!pip install ultralytics ethos-u-vela -q

# Mount Google Drive
from google.colab import drive
drive.mount('/content/drive')
# Find best.pt in Google Drive
import glob
pt_files = glob.glob('/content/drive/**/**/best.pt', recursive=True)
for f in pt_files:
    print(f)

# Export to INT8 TFLite at 192x192
# UPDATE the path below if the search above finds a different location
from ultralytics import YOLO

PT_PATH = pt_files[0]  # uses first match — change if needed
print(f"Using: {PT_PATH}")

model = YOLO(PT_PATH)
model.export(
    format='tflite',
    imgsz=192,
    int8=True,
)
print("Export done!")

# Vela compile for Ethos-U55 (Grove Vision AI V2)
import glob, os

tflite_files = glob.glob(os.path.dirname(PT_PATH) + '/*int8*.tflite') + \
               glob.glob(os.path.dirname(PT_PATH) + '/**/*int8*.tflite', recursive=True)
if not tflite_files:
    tflite_files = glob.glob(os.path.dirname(PT_PATH) + '/**/*.tflite', recursive=True)

tflite_path = tflite_files[0]
print(f"Compiling: {tflite_path}")
os.makedirs('output_vela', exist_ok=True)

!vela --accelerator-config ethos-u55-64 \
    --system-config Ethos_U55_High_End_Embedded \
    --memory-mode Shared_Sram \
    --output-dir ./output_vela \
    "{tflite_path}"

# Download the Vela-compiled model
import glob
from google.colab import files

vela_models = glob.glob('output_vela/*_vela.tflite')
print(f"Vela model: {vela_models[0]}")
print(f"Size: {os.path.getsize(vela_models[0]) / (1024*1024):.1f} MB")

files.download(vela_models[0])


from google.colab import drive
drive.mount('/content/gdrive')


# Compare output shapes between the two models
import numpy as np
import tensorflow.lite as tflite

# Load NHWC model
interp = tflite.Interpreter(model_path='/content/gdrive/MyDrive/cleanbot-weights/best_float32_nhwc.tflite')
interp.allocate_tensors()
inp = interp.get_input_details()
out = interp.get_output_details()

print("=== NHWC (float32) MODEL ===")
print(f"Input:  shape={inp[0]['shape']}, dtype={inp[0]['dtype']}, name='{inp[0]['name']}'")
for i, o in enumerate(out):
    print(f"Output[{i}]: shape={o['shape']}, dtype={o['dtype']}, name='{o['name']}'")

# Load INT8 CHW model
interp2 = tflite.Interpreter(model_path='/content/gdrive/MyDrive/cleanbot-weights/best_int8.tflite')
interp2.allocate_tensors()
inp2 = interp2.get_input_details()
out2 = interp2.get_output_details()

print("\n=== INT8 (CHW) MODEL ===")
print(f"Input:  shape={inp2[0]['shape']}, dtype={inp2[0]['dtype']}, name='{inp2[0]['name']}'")
for i, o in enumerate(out2):
    print(f"Output[{i}]: shape={o['shape']}, dtype={o['dtype']}, name='{o['name']}'")

# Run both on same test image
from PIL import Image
import glob

imgs = glob.glob('/content/dataset/project/images/*.jpg')[:1]
if not imgs:
    imgs = glob.glob('/content/gdrive/MyDrive/next-ne-dataset/**/*.jpg', recursive=True)[:1]
img_path = imgs[0]
print(f"\nTest image: {img_path}")

img = Image.open(img_path).resize((320, 320))
img_np = np.array(img, dtype=np.float32) / 255.0

# NHWC model: input is (1, 320, 320, 3)
nhwc_input = np.expand_dims(img_np, 0)
interp.set_tensor(inp[0]['index'], nhwc_input)
interp.invoke()
nhwc_out = [interp.get_tensor(o['index']) for o in out]
print(f"\nNHWC output shapes: {[x.shape for x in nhwc_out]}")
print(f"NHWC output[0] min={nhwc_out[0].min():.4f} max={nhwc_out[0].max():.4f} mean={nhwc_out[0].mean():.4f}")

# INT8 model: input is (1, 3, 320, 320)
chw_input = np.expand_dims(np.transpose(img_np, (2, 0, 1)), 0)
# INT8 needs quantized input
inp2_details = inp2[0]
if inp2_details['dtype'] == np.int8:
    scale, zp = inp2_details['quantization']
    chw_input = (chw_input / scale + zp).astype(np.int8)
elif inp2_details['dtype'] == np.uint8:
    scale, zp = inp2_details['quantization']
    chw_input = (chw_input / scale + zp).astype(np.uint8)
interp2.set_tensor(inp2[0]['index'], chw_input)
interp2.invoke()
chw_out = [interp2.get_tensor(o['index']) for o in out2]
print(f"\nCHW output shapes: {[x.shape for x in chw_out]}")
print(f"CHW output[0] min={chw_out[0].min():.4f} max={chw_out[0].max():.4f} mean={chw_out[0].mean():.4f}")


import os
print(os.listdir('/content/gdrive/MyDrive/cleanbot-weights/'))


# Find a test image and compare outputs
import numpy as np, glob
from PIL import Image
import tensorflow.lite as tflite

# Find images in drive
imgs = glob.glob('/content/gdrive/MyDrive/next-ne-dataset/**/*.jpg', recursive=True)[:1]
if not imgs:
    imgs = glob.glob('/content/gdrive/MyDrive/**/*.jpg', recursive=True)[:1]
print(f"Found {len(imgs)} images")
if not imgs:
    print("No images found - upload one or extract dataset first")
    raise SystemExit

img_path = imgs[0]
print(f"Test: {img_path}")
img = Image.open(img_path).resize((320, 320))
img_np = np.array(img, dtype=np.float32) / 255.0

# NHWC model
interp = tflite.Interpreter(model_path='/content/gdrive/MyDrive/cleanbot-weights/best_float32_nhwc.tflite')
interp.allocate_tensors()
inp = interp.get_input_details()
out = interp.get_output_details()
interp.set_tensor(inp[0]['index'], np.expand_dims(img_np, 0))
interp.invoke()
nhwc_raw = interp.get_tensor(out[0]['index'])

# INT8 CHW model
interp2 = tflite.Interpreter(model_path='/content/gdrive/MyDrive/cleanbot-weights/best_int8.tflite')
interp2.allocate_tensors()
inp2 = interp2.get_input_details()
out2 = interp2.get_output_details()
chw_input = np.expand_dims(np.transpose(img_np, (2, 0, 1)), 0).astype(np.float32)
interp2.set_tensor(inp2[0]['index'], chw_input)
interp2.invoke()
chw_raw = interp2.get_tensor(out2[0]['index'])

# Compare raw outputs
print(f"\nNHWC raw: min={nhwc_raw.min():.4f} max={nhwc_raw.max():.4f} mean={nhwc_raw.mean():.6f}")
print(f"CHW  raw: min={chw_raw.min():.4f} max={chw_raw.max():.4f} mean={chw_raw.mean():.6f}")

# Apply sigmoid to class scores (indices 4:7) and check max confidence
from scipy.special import expit as sigmoid

nhwc_scores = sigmoid(nhwc_raw[0, 4:7, :])  # (3, 2100)
chw_scores = sigmoid(chw_raw[0, 4:7, :])

print(f"\nNHWC max class conf: {nhwc_scores.max():.4f}")
print(f"CHW  max class conf: {chw_scores.max():.4f}")

# Top 5 detections from each
for name, scores, raw in [("NHWC", nhwc_scores, nhwc_raw), ("CHW", chw_scores, chw_raw)]:
    max_per_anchor = scores.max(axis=0)  # (2100,)
    top5 = np.argsort(max_per_anchor)[-5:][::-1]
    print(f"\n{name} top 5 detections:")
    classes = ['ball', 'blue bucket', 'orange bucket']
    for idx in top5:
        cls = scores[:, idx].argmax()
        conf = scores[cls, idx]
        print(f"  anchor {idx}: {classes[cls]} conf={conf:.4f}")


!pip install ultralytics onnx2tf -q


# Strategy: export to TF SavedModel via Ultralytics (NHWC natively),
# then convert to TFLite ourselves — preserves 0-1 normalized coords
from ultralytics import YOLO

model = YOLO('/content/gdrive/MyDrive/cleanbot-weights/best.pt')
model.model.eval()

# Export to TF SavedModel — this produces NHWC input natively
model.export(format='saved_model', imgsz=320)


import tensorflow as tf
import numpy as np

# Convert SavedModel to TFLite
converter = tf.lite.TFLiteConverter.from_saved_model(
    '/content/gdrive/MyDrive/cleanbot-weights/best_saved_model'
)
tflite_model = converter.convert()

out_path = '/content/gdrive/MyDrive/cleanbot-weights/best_nhwc_v2.tflite'
with open(out_path, 'wb') as f:
    f.write(tflite_model)
print(f"Saved: {out_path} ({len(tflite_model)/1e6:.1f} MB)")

# Test it
interp = tf.lite.Interpreter(model_content=tflite_model)
interp.allocate_tensors()
inp = interp.get_input_details()
out = interp.get_output_details()
print(f"Input: {inp[0]['shape']}, {inp[0]['dtype']}")
print(f"Output: {out[0]['shape']}, {out[0]['dtype']}")

# Run on a test image
from PIL import Image
import glob
imgs = glob.glob('/content/gdrive/MyDrive/next-ne-dataset/**/*.jpg', recursive=True)[:1]
img = Image.open(imgs[0]).resize((320, 320))
img_np = np.array(img, dtype=np.float32) / 255.0
interp.set_tensor(inp[0]['index'], np.expand_dims(img_np, 0))
interp.invoke()
raw = interp.get_tensor(out[0]['index'])
print(f"\nOutput raw: min={raw.min():.4f} max={raw.max():.4f}")
print(f"Bbox range (rows 0-3): min={raw[0,:4,:].min():.4f} max={raw[0,:4,:].max():.4f}")
print(f"Class scores (rows 4-6): min={raw[0,4:,:].min():.4f} max={raw[0,4:,:].max():.4f}")


import glob, os

# Find any jpg on drive
for d in ['/content/gdrive/MyDrive/next-ne-dataset', '/content/gdrive/MyDrive/cleanbot', '/content/gdrive/MyDrive']:
    imgs = glob.glob(os.path.join(d, '**/*.jpg'), recursive=True)[:3]
    if imgs:
        print(f"Found {len(imgs)} in {d}")
        for i in imgs: print(f"  {i}")
        break

# Also check /content for extracted dataset
imgs2 = glob.glob('/content/dataset/**/*.jpg', recursive=True)[:3]
if imgs2:
    print(f"\nFound in /content/dataset:")
    for i in imgs2: print(f"  {i}")


# Extract dataset to get proper test images
import zipfile
zip_path = '/content/gdrive/MyDrive/next-ne-dataset/next-ne-dataset/Copy of data.zip'
if os.path.exists(zip_path):
    with zipfile.ZipFile(zip_path) as z:
        z.extractall('/content/dataset')
    print("Extracted")
    imgs = glob.glob('/content/dataset/**/*.jpg', recursive=True)[:3]
    for i in imgs: print(i)
else:
    print(f"Zip not found at {zip_path}")
    # List what's in next-ne-dataset
    for d in ['/content/gdrive/MyDrive/next-ne-dataset']:
        if os.path.exists(d):
            for root, dirs, files in os.walk(d):
                for f in files[:5]:
                    print(os.path.join(root, f))


import numpy as np, tensorflow as tf
from PIL import Image

img = Image.open('/content/dataset/project/images/20260711_161701.jpg.6tfj743r.ingestion-5759d4ffb8-w7hgq.jpg').resize((320,320))
img_np = np.array(img, dtype=np.float32) / 255.0

# Test new v2 model
interp = tf.lite.Interpreter(model_path='/content/gdrive/MyDrive/cleanbot-weights/best_nhwc_v2.tflite')
interp.allocate_tensors()
inp = interp.get_input_details()
out = interp.get_output_details()
interp.set_tensor(inp[0]['index'], np.expand_dims(img_np, 0))
interp.invoke()
raw = interp.get_tensor(out[0]['index'])

print(f"Output shape: {raw.shape}")
print(f"Bbox (rows 0-3) range: min={raw[0,:4,:].min():.4f} max={raw[0,:4,:].max():.4f}")
print(f"Class scores (rows 4-6) range: min={raw[0,4:,:].min():.4f} max={raw[0,4:,:].max():.4f}")

# Compare with old NHWC model
interp2 = tf.lite.Interpreter(model_path='/content/gdrive/MyDrive/cleanbot-weights/best_float32_nhwc.tflite')
interp2.allocate_tensors()
inp2 = interp2.get_input_details()
out2 = interp2.get_output_details()
interp2.set_tensor(inp2[0]['index'], np.expand_dims(img_np, 0))
interp2.invoke()
raw2 = interp2.get_tensor(out2[0]['index'])

print(f"\nOld NHWC bbox range: min={raw2[0,:4,:].min():.4f} max={raw2[0,:4,:].max():.4f}")
print(f"Old NHWC class range: min={raw2[0,4:,:].min():.4f} max={raw2[0,4:,:].max():.4f}")

# Check if new model coords are 0-1 or absolute
if raw[0,:4,:].max() > 2.0:
    print("\n⚠️  v2 coords are ABSOLUTE (pixel scale)")
else:
    print("\n✅ v2 coords are NORMALIZED (0-1)")


# Approach: load PyTorch model, add NHWC→CHW transpose at input,
# export to ONNX with NHWC input, then convert to TFLite
# This preserves the original 0-1 normalized output format

import torch
import torch.nn as nn
from ultralytics import YOLO

class NHWCWrapper(nn.Module):
    def __init__(self, model):
        super().__init__()
        self.model = model

    def forward(self, x):
        # x is NHWC (batch, H, W, C) → convert to NCHW (batch, C, H, W)
        x = x.permute(0, 3, 1, 2)
        return self.model(x)

yolo = YOLO('/content/gdrive/MyDrive/cleanbot-weights/best.pt')
yolo.model.eval()

wrapper = NHWCWrapper(yolo.model)
wrapper.eval()

# Test with dummy input
dummy = torch.randn(1, 320, 320, 3)
with torch.no_grad():
    out = wrapper(dummy)
print(f"Output shape: {out.shape}")

# Export to ONNX with NHWC input
torch.onnx.export(
    wrapper,
    dummy,
    '/content/best_nhwc.onnx',
    input_names=['images'],
    output_names=['output0'],
    opset_version=13,
    dynamic_axes=None
)
print("ONNX exported with NHWC input")

# Convert ONNX to TFLite via onnx2tf — but this time the model itself
# handles the transpose, so onnx2tf won't mess with the output format


import torch
import torch.nn as nn
from ultralytics import YOLO

class NHWCWrapper(nn.Module):
    def __init__(self, yolo_model):
        super().__init__()
        self.model = yolo_model

    def forward(self, x):
        # x: NHWC (1, 320, 320, 3) → NCHW (1, 3, 320, 320)
        x = x.permute(0, 3, 1, 2)
        out = self.model(x)
        if isinstance(out, (tuple, list)):
            return out[0]
        return out

yolo = YOLO('/content/gdrive/MyDrive/cleanbot-weights/best.pt')
yolo.model.eval()
wrapper = NHWCWrapper(yolo.model)
wrapper.eval()

dummy = torch.randn(1, 320, 320, 3)
with torch.no_grad():
    out = wrapper(dummy)
print(f"Output shape: {out.shape}")
print(f"Output range: min={out.min():.4f} max={out.max():.4f}")

# Export ONNX
torch.onnx.export(
    wrapper, dummy,
    '/content/best_nhwc.onnx',
    input_names=['images'],
    output_names=['output0'],
    opset_version=13
)
print("ONNX saved")

# Convert to TFLite directly (not via onnx2tf)
# Use tf converter from the ONNX
import subprocess
result = subprocess.run([
    'python', '-m', 'onnx2tf',
    '-i', '/content/best_nhwc.onnx',
    '-o', '/content/nhwc_saved_model',
    '-oiqt'  # output int8 quantized tflite
], capture_output=True, text=True)
print(result.stdout[-500:] if result.stdout else "")
print(result.stderr[-500:] if result.stderr else "")


# Check: does the original Ultralytics ONNX have 0-1 coords?
import onnxruntime as ort
import numpy as np
from PIL import Image

sess = ort.InferenceSession('/content/gdrive/MyDrive/cleanbot-weights/best.onnx')
inp_name = sess.get_inputs()[0].name
inp_shape = sess.get_inputs()[0].shape
out_name = sess.get_outputs()[0].name
print(f"ONNX input: {inp_name} {inp_shape}")

img = Image.open('/content/dataset/project/images/20260711_161701.jpg.6tfj743r.ingestion-5759d4ffb8-w7hgq.jpg').resize((320,320))
img_np = np.array(img, dtype=np.float32) / 255.0
# CHW for ONNX
chw = np.expand_dims(np.transpose(img_np, (2,0,1)), 0)

result = sess.run([out_name], {inp_name: chw})[0]
print(f"ONNX output shape: {result.shape}")
print(f"Bbox (0:4) range: min={result[0,:4,:].min():.4f} max={result[0,:4,:].max():.4f}")
print(f"Class (4:7) range: min={result[0,4:,:].min():.4f} max={result[0,4:,:].max():.4f}")


# Retest INT8 CHW model on actual training image
import numpy as np, tensorflow as tf
from PIL import Image

img = Image.open('/content/dataset/project/images/20260711_161701.jpg.6tfj743r.ingestion-5759d4ffb8-w7hgq.jpg').resize((320,320))
img_np = np.array(img, dtype=np.float32) / 255.0

interp = tf.lite.Interpreter(model_path='/content/gdrive/MyDrive/cleanbot-weights/best_int8.tflite')
interp.allocate_tensors()
inp = interp.get_input_details()[0]
out = interp.get_output_details()[0]

chw = np.expand_dims(np.transpose(img_np, (2,0,1)), 0).astype(np.float32)
interp.set_tensor(inp['index'], chw)
interp.invoke()
raw = interp.get_tensor(out['index'])

print(f"INT8 CHW on training image:")
print(f"Bbox (0:4): min={raw[0,:4,:].min():.4f} max={raw[0,:4,:].max():.4f}")
print(f"Class (4:7): min={raw[0,4:,:].min():.4f} max={raw[0,4:,:].max():.4f}")


# Check INT8 quantization params on output
import tensorflow as tf
interp = tf.lite.Interpreter(model_path='/content/gdrive/MyDrive/cleanbot-weights/best_int8.tflite')
interp.allocate_tensors()
out = interp.get_output_details()[0]
print(f"Output dtype: {out['dtype']}")
print(f"Output quantization: {out['quantization']}")
print(f"Output quantization_parameters: {out['quantization_parameters']}")

# The output is dequantized float32 but the internal INT8 range limits it
# So the INT8 model literally can't output values > ~1 for bboxes
# This means it was quantized assuming bbox values are 0-1 (normalized)
# But the actual model outputs 0-320 absolute coords
# → INT8 quantization is BROKEN for this model

# The float32 NHWC model is correct (absolute coords 0-320)
# We just need Edge Impulse to handle it properly
print("\nConclusion: INT8 model has broken bbox outputs due to bad quantization range")
print("The float32 NHWC model is actually the correct one")
print("Edge Impulse 'YOLOv11 absolute values' should work if it's properly supported")

# # Train Model to Detect Road Obstucals
# 
# We use Ultralytics to train YOLOv11 object detection models on our dataset. We collected

# ## Prepare data and scripts

!nvidia-smi

from google.colab import drive
# Connect to Google Drive, such that we can access to our dataset.
drive.mount('/content/gdrive')

import os

# Create the directory if it doesn't exist
!mkdir -p /content/cleanbot
!ls /content/

import os
print("train labels:", len(os.listdir('/content/data/train/labels')))
print("val labels:", len(os.listdir('/content/data/validation/labels')))

# copy dataset to the experiment directory
!cp "/content/gdrive/MyDrive/next-ne-dataset/next-ne-dataset/Copy of data.zip" /content/cleanbot/data.zip
!ls /content/cleanbot/

!unzip /content/cleanbot/data.zip -d /content/cleanbot/dataset

# copy Python scripts to the experiment directory
!cp "/content/gdrive/MyDrive/next-ne-dataset/next-ne-dataset/Copy of train_val_split.py" /content/cleanbot/train_val_split.py
!ls /content/cleanbot/

print('Searching for train_val_split.py and data.yaml in Google Drive. This might take a moment...')
!find /content/gdrive/MyDrive -name 'train_val_split.py'
!find /content/gdrive/MyDrive -name 'data.yaml'

# Split dataset into train and testing subsets at a ratio of 8:2 (common ratio in other image tasks).
!python /content/cleanbot/train_val_split.py --datapath='/content/cleanbot/dataset/project' --train_pct=0.8

# Read class names from classes.txt
with open('/content/cleanbot/dataset/project/classes.txt', 'r') as f:
    class_names = f.read().splitlines()

nc = len(class_names) # Number of classes

# Create data.yaml content
data_yaml_content = f"""
train: /content/data/train/images
val: /content/data/validation/images

nc: {nc}
names: {class_names}
"""

# Save data.yaml to the experiment directory
with open('/content/cleanbot/data.yaml', 'w') as f:
    f.write(data_yaml_content)

print('data.yaml created successfully:')
!cat /content/cleanbot/data.yaml

# ## Set up environment

!pip install ultralytics

# ## Train Model
# 
# There are several YOLOv11 models with various sizes: yolo11s.pt (smallest), yolo11n.pt, yolo11m.pt, yolo11l.pt, and yolo11x.pt (largest). Larger models have higher accuracy but run slower, while smaller models run faster but have lower accuracy. We choose YOLO11n.pt to balance the performance and detection speed.

# Train a yolo11 object detection model on our dataset.
!yolo detect train data=/content/cleanbot/data.yaml model=yolo11n.pt epochs=100 imgsz=640

!cp -r /content/runs/detect/train-3/weights /content/gdrive/MyDrive/cleanbot-weights

# Export to TFLite for Edge Impulse upload
!yolo export model=runs/detect/train-3/weights/best.pt format=tflite
print('TFLite model exported. Download from runs/detect/train-3/weights/')

print('Saving the updated train_val_split.py script...')
script_content = """
# Split between train and val folders

from pathlib import Path
import random
import os
import sys
import shutil
import argparse


# Define and parse user input arguments

parser = argparse.ArgumentParser()
parser.add_argument('--datapath', help='Path to data folder containing image and annotation files',
                    required=True)
parser.add_argument('--train_pct', help='Ratio of images to go to train folder; \
                    the rest go to validation folder (example: ".8")',
                    default=.8)

args = parser.parse_args()

data_path = args.datapath
train_percent = float(args.train_pct)

# Check for valid entries
if not os.path.isdir(data_path):
   print('Directory specified by --datapath not found. Verify the path is correct (and uses double back slashes if on Windows) and try again.')
   sys.exit(0)
if train_percent < .01 or train_percent > 0.99:
   print('Invalid entry for train_pct. Please enter a number between .01 and .99.')
   sys.exit(0)
val_percent = 1 - train_percent

# Define path to input dataset
input_image_path = os.path.join(data_path,'images')
input_label_path = os.path.join(data_path,'labels')

# Define paths to image and annotation folders
cwd = os.getcwd()
train_img_path = os.path.join(cwd,'data/train/images')
train_txt_path = os.path.join(cwd,'data/train/labels')
val_img_path = os.path.join(cwd,'data/validation/images')
val_txt_path = os.path.join(cwd,'data/validation/labels')

# Remove existing folders to ensure a clean split
for dir_path in [train_img_path, train_txt_path, val_img_path, val_txt_path]:
   if os.path.exists(dir_path):
      shutil.rmtree(dir_path)
      print(f'Removed existing folder at {dir_path}.')

# Create folders if they don't already exist
for dir_path in [train_img_path, train_txt_path, val_img_path, val_txt_path]:
   os.makedirs(dir_path)
   print(f'Created folder at {dir_path}.')


# Get list of all images and annotation files
img_file_list = [path for path in Path(input_image_path).rglob('*')]
# Note: txt_file_list is only used for initial count, not for lookup in the loop below
# because the label filename is derived from the image filename's UUID part.
txt_file_list = [path for path in Path(input_label_path).rglob('*')]

print(f'Number of image files: {len(img_file_list)}')
print(f'Number of annotation files: {len(txt_file_list)}')

# Determine number of files to move to each folder
file_num = len(img_file_list)
train_num = int(file_num*train_percent)
val_num = file_num - train_num
print('Images moving to train: %d' % train_num)
print('Images moving to validation: %d' % val_num)

# Select files randomly and copy them to train or val folders
for i, set_num in enumerate([train_num, val_num]):
  for ii in range(set_num):
    img_path = random.choice(img_file_list)
    img_fn = img_path.name

    # Extract the UUID part from the image filename
    # Assuming the UUID is the first part before the first '.'
    # This logic correctly handles names like 'UUID.jpg.suffix.jpg' or 'UUID_part.jpeg.suffix.jpg'
    # to find a corresponding label like 'UUID.txt' or 'UUID_part.txt'
    uuid_part = img_fn.split('.')[0]
    txt_fn = uuid_part + '.txt'
    txt_path = os.path.join(input_label_path,txt_fn)

    if i == 0: # Copy first set of files to train folders
      new_img_path, new_txt_path = train_img_path, train_txt_path
    elif i == 1: # Copy second set of files to the validation folders
      new_img_path, new_txt_path = val_img_path, val_txt_path

    shutil.copy(img_path, os.path.join(new_img_path,img_fn))
    if os.path.exists(txt_path): # Check if the corresponding txt file exists
      shutil.copy(txt_path,os.path.join(new_txt_path,txt_fn))
    else:
      print(f"Warning: No label file found for {img_fn} at {txt_path}. This image might be a background image or has a missing annotation.")

    img_file_list.remove(img_path)
"""

with open('/content/cleanbot/train_val_split.py', 'w') as f:
    f.write(script_content)

print('train_val_split.py saved to /content/cleanbot/. Re-running data splitting.')

# Re-run data splitting with the updated script
!python /content/cleanbot/train_val_split.py --datapath='/content/cleanbot/dataset/project' --train_pct=0.8

# Verify the contents of the label directories after re-splitting
print('Contents of /content/data/train/labels:')
!ls -la /content/data/train/labels

print('\nContents of /content/data/validation/labels:')
!ls -la /content/data/validation/labels

# Read class names from classes.txt
with open('/content/cleanbot/dataset/project/classes.txt', 'r') as f:
    class_names = f.read().splitlines()

nc = len(class_names) # Number of classes

# Create data.yaml content
data_yaml_content = f"""
train: /content/data/train
val: /content/data/validation

nc: {nc}
names: {class_names}
"""

# Save data.yaml to the experiment directory
with open('/content/cleanbot/data.yaml', 'w') as f:
    f.write(data_yaml_content)

print('data.yaml created successfully:')
!cat /content/cleanbot/data.yaml

import os, re

# Fix image filenames: strip the ".XXX.ingestion-XXX.jpg" suffix
# e.g. "UUID.jpg.6tfj72u8.ingestion-5759d4ffb8-tj7rs.jpg" -> "UUID.jpg"
for split_dir in ['/content/data/train/images', '/content/data/validation/images']:
    renamed = 0
    for fn in os.listdir(split_dir):
        # Match: name.ext.random.ingestion-xxx.ext
        match = re.match(r'^(.+?\.(jpg|jpeg|png))\..+$', fn, re.IGNORECASE)
        if match:
            new_fn = match.group(1)
            old_path = os.path.join(split_dir, fn)
            new_path = os.path.join(split_dir, new_fn)
            os.rename(old_path, new_path)
            renamed += 1
    print(f'{split_dir}: renamed {renamed} files')

# Verify a few
sample = os.listdir('/content/data/train/images')[:3]
print('Sample image names:', sample)
sample_labels = os.listdir('/content/data/train/labels')[:3]
print('Sample label names:', sample_labels)

import glob
for f in glob.glob('/content/data/**/*.cache', recursive=True):
    os.remove(f)
    print(f'Removed {f}')

import os
# Check current state
imgs = os.listdir('/content/data/train/images')[:5]
lbls = os.listdir('/content/data/train/labels')[:5]
print(f"Train images: {len(os.listdir('/content/data/train/images'))}")
print(f"Train labels: {len(os.listdir('/content/data/train/labels'))}")
print(f"Sample images: {imgs}")
print(f"Sample labels: {lbls}")
# Check if stems match
img_stems = {os.path.splitext(f)[0] for f in os.listdir('/content/data/train/images')}
lbl_stems = {os.path.splitext(f)[0] for f in os.listdir('/content/data/train/labels')}
print(f"\nMatching stems: {len(img_stems & lbl_stems)} / {len(img_stems)} images")

# Retry training the yolo11 object detection model
!yolo detect train data=/content/cleanbot/data.yaml model=yolo11n.pt epochs=100 imgsz=640

# Retry training the yolo11 object detection model
!yolo detect train data=/content/cleanbot/data.yaml model=yolo11n.pt epochs=100 imgsz=640

print('Contents of /content/data/train/labels:')
!ls -la /content/data/train/labels

print('\nContents of /content/data/validation/labels:')
!ls -la /content/data/validation/labels

# continue improving model
# !yolo detect train resume=true data=/content/cleanbot/data.yaml model=runs/detect/train4/weights/best.pt epochs=100 imgsz=640

# ## Download model

# Export to TFLite for Edge Impulse
!yolo export model=/content/runs/detect/train-3/weights/best.pt format=tflite
print('Done! Download the .tflite from /content/runs/detect/train-3/weights/')

import tensorflow as tf
import shutil, os

# Verify the float32 NHWC model
interpreter = tf.lite.Interpreter(model_path='/content/saved_model_int8/best_float32.tflite')
interpreter.allocate_tensors()
inp = interpreter.get_input_details()[0]
out = interpreter.get_output_details()
print(f"Input: shape={inp['shape']}, dtype={inp['dtype']}")
for o in out:
    print(f"Output: {o['name']} shape={o['shape']} dtype={o['dtype']}")

# Copy to Drive
shutil.copy('/content/saved_model_int8/best_float32.tflite',
            '/content/gdrive/MyDrive/cleanbot-weights/best_float32_nhwc.tflite')

sz = os.path.getsize('/content/gdrive/MyDrive/cleanbot-weights/best_float32_nhwc.tflite')
print(f'\nSaved to Drive: best_float32_nhwc.tflite ({sz/1e6:.1f} MB)')
print('Upload this to Edge Impulse — input shape (1, 320, 320, 3) NHWC.')

# Verify the INT8 model actually detects objects (one image at a time for TFLite)
from ultralytics import YOLO
import os

model = YOLO('/content/gdrive/MyDrive/cleanbot-weights/best_int8.tflite', task='detect')

val_imgs = os.listdir('/content/dataset/project/images')[:5]
for f in val_imgs:
    path = f'/content/dataset/project/images/{f}'
    results = model.predict(path, imgsz=320, conf=0.25)
    r = results[0]
    print(f'{f}: {len(r.boxes)} detections')
    for box in r.boxes:
        cls = int(box.cls[0])
        conf = float(box.conf[0])
        name = r.names[cls]
        print(f'  -> {name} ({conf:.2f})')

!mkdir -p /content/cleanbot/export/model
!mkdir -p /content/cleanbot/export/dataset
!mkdir -p /content/cleanbot/export/train-logs

# copy the trained model and separated dataset to the export folder
!cp /content/runs/detect/train4/weights/best.pt /content/cleanbot/export/model/best_cleanbot.pt
# split dataset
!cp -rf /content/cleanbot/data/train /content/cleanbot/export/dataset/
!cp -rf /content/cleanbot/data/validation /content/cleanbot/export/dataset/
# training logs
!cp -rf /content/runs/detect/train4 /content/cleanbot/export/train-logs

# ## Train NanoDet-Plus
# 
# Lighter model optimized for edge devices (Raspberry Pi). Uses the same dataset as YOLO above.

!pip install git+https://github.com/RangiLyu/nanodet.git pytorch-lightning -q 2>&1 | tail -5

import os
if not os.path.exists('/content/gdrive/MyDrive'):
    from google.colab import drive
    drive.mount('/content/gdrive')

# Extract dataset
if not os.path.exists('/content/dataset/project/images'):
    !cp "/content/gdrive/MyDrive/next-ne-dataset/next-ne-dataset/Copy of data.zip" /content/data.zip
    !unzip -qo /content/data.zip -d /content/dataset

print(f"Images: {len(os.listdir('/content/dataset/project/images'))}")
print("Setup done")

# Split dataset into train/val (80/20) and fix filenames
import os, shutil, random

src_imgs = '/content/dataset/project/images'
src_lbls = '/content/dataset/project/labels'

for split in ['train', 'val']:
    os.makedirs(f'/content/data/{split}/images', exist_ok=True)
    os.makedirs(f'/content/data/{split}/labels', exist_ok=True)

all_imgs = [f for f in os.listdir(src_imgs) if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
random.seed(42)
random.shuffle(all_imgs)

split_idx = int(len(all_imgs) * 0.8)
splits = {'train': all_imgs[:split_idx], 'val': all_imgs[split_idx:]}

for split, files in splits.items():
    for f in files:
        shutil.copy(f'{src_imgs}/{f}', f'/content/data/{split}/images/{f}')
        lbl = os.path.splitext(f)[0] + '.txt'
        lbl_path = f'{src_lbls}/{lbl}'
        if os.path.exists(lbl_path):
            shutil.copy(lbl_path, f'/content/data/{split}/labels/{lbl}')

print(f"Train: {len(splits['train'])} images")
print(f"Val: {len(splits['val'])} images")

# Re-split with proper label matching
# Image: "20260711_202946.jpg.6tfj671f.ingestion-5759d4ffb8-8lvsf.jpg"
# Label: "20260711_202946.txt"
# Pattern: label_stem matches everything before the first ".jpg" in image filename

import os, shutil, random, re, json
from PIL import Image

CLASS_NAMES = ['ball', 'blue bucket', 'orange bucket']

src_imgs = '/content/dataset/project/images'
src_lbls = '/content/dataset/project/labels'

# Build label stem -> label file mapping
lbl_map = {}
for l in os.listdir(src_lbls):
    if l.endswith('.txt') and l != 'classes.txt':
        stem = os.path.splitext(l)[0]
        lbl_map[stem] = l

# Build image -> label mapping via first ".jpg" extraction
img_to_lbl = {}
all_imgs = [f for f in os.listdir(src_imgs) if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
for img in all_imgs:
    # Extract base name: everything before first ".jpg"
    idx = img.lower().find('.jpg')
    if idx >= 0:
        stem = img[:idx]
    else:
        stem = os.path.splitext(img)[0]
    if stem in lbl_map:
        img_to_lbl[img] = lbl_map[stem]

print(f"Total images: {len(all_imgs)}")
print(f"Matched with labels: {len(img_to_lbl)}")

# Split and copy
random.seed(42)
matched_imgs = list(img_to_lbl.keys())
random.shuffle(matched_imgs)
split_idx = int(len(matched_imgs) * 0.8)

for split in ['train', 'val']:
    for sub in ['images', 'labels']:
        os.makedirs(f'/content/data/{split}/{sub}', exist_ok=True)

splits = {'train': matched_imgs[:split_idx], 'val': matched_imgs[split_idx:]}

for split, files in splits.items():
    for img_f in files:
        shutil.copy(f'{src_imgs}/{img_f}', f'/content/data/{split}/images/{img_f}')
        lbl_f = img_to_lbl[img_f]
        # Save label with same stem as image
        img_stem = os.path.splitext(img_f)[0]
        shutil.copy(f'{src_lbls}/{lbl_f}', f'/content/data/{split}/labels/{img_stem}.txt')

print(f"Train: {len(splits['train'])} | Val: {len(splits['val'])}")

# Now convert to COCO JSON
def yolo_to_coco(img_dir, label_dir, output_json):
    images, annotations = [], []
    ann_id = 0
    img_files = sorted([f for f in os.listdir(img_dir) if f.lower().endswith(('.jpg', '.jpeg', '.png'))])

    for img_id, img_fn in enumerate(img_files):
        img = Image.open(f'{img_dir}/{img_fn}')
        w, h = img.size
        images.append({'id': img_id, 'file_name': img_fn, 'width': w, 'height': h})

        lbl_fn = os.path.splitext(img_fn)[0] + '.txt'
        lbl_path = f'{label_dir}/{lbl_fn}'
        if os.path.exists(lbl_path):
            with open(lbl_path) as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) < 5: continue
                    cls_id = int(parts[0])
                    cx, cy, bw, bh = map(float, parts[1:5])
                    x = (cx - bw/2) * w
                    y = (cy - bh/2) * h
                    annotations.append({
                        'id': ann_id, 'image_id': img_id, 'category_id': cls_id + 1,
                        'bbox': [x, y, bw * w, bh * h], 'area': bw * w * bh * h, 'iscrowd': 0
                    })
                    ann_id += 1

    coco = {'images': images, 'annotations': annotations,
            'categories': [{'id': i+1, 'name': n} for i, n in enumerate(CLASS_NAMES)]}
    with open(output_json, 'w') as f:
        json.dump(coco, f)
    print(f'{output_json}: {len(images)} images, {len(annotations)} annotations')

os.makedirs('/content/nanodet', exist_ok=True)
yolo_to_coco('/content/data/train/images', '/content/data/train/labels', '/content/nanodet/train.json')
yolo_to_coco('/content/data/val/images', '/content/data/val/labels', '/content/nanodet/val.json')

# Write NanoDet-Plus config with aux_head
config = """
save_dir: /content/nanodet/workspace
class_names: ['ball', 'blue bucket', 'orange bucket']
model:
  weight_averager:
    name: ExpMovingAverager
    decay: 0.9998
  arch:
    name: NanoDetPlus
    detach_epoch: 10
    backbone:
      name: ShuffleNetV2
      model_size: 1.0x
      out_stages: [2, 3, 4]
      activation: LeakyReLU
    fpn:
      name: GhostPAN
      in_channels: [116, 232, 464]
      out_channels: 96
      kernel_size: 5
      num_extra_level: 1
      use_depthwise: True
      activation: LeakyReLU
    aux_head:
      name: SimpleConvHead
      num_classes: 3
      input_channel: 192
      feat_channels: 192
      stacked_convs: 2
      strides: [8, 16, 32, 64]
      activation: LeakyReLU
      reg_max: 7
    head:
      name: NanoDetPlusHead
      num_classes: 3
      input_channel: 96
      feat_channels: 96
      stacked_convs: 2
      kernel_size: 5
      strides: [8, 16, 32, 64]
      activation: LeakyReLU
      reg_max: 7
      norm_cfg:
        type: BN
      loss:
        loss_qfl:
          name: QualityFocalLoss
          use_sigmoid: True
          beta: 2.0
          loss_weight: 1.0
        loss_dfl:
          name: DistributionFocalLoss
          loss_weight: 0.25
        loss_bbox:
          name: GIoULoss
          loss_weight: 2.0
data:
  train:
    name: CocoDataset
    img_path: /content/data/train/images
    ann_path: /content/nanodet/train.json
    input_size: [320, 320]
    keep_ratio: False
    pipeline:
      perspective: 0
      scale: [0.6, 1.4]
      stretch: [[0.8, 1.2], [0.8, 1.2]]
      rotation: 0
      shear: 0
      translate: 0.2
      flip: 0.5
      brightness: 0.2
      contrast: [0.6, 1.4]
      saturation: [0.5, 1.2]
      normalize: [[103.53, 116.28, 123.675], [57.375, 57.12, 58.395]]
  val:
    name: CocoDataset
    img_path: /content/data/val/images
    ann_path: /content/nanodet/val.json
    input_size: [320, 320]
    keep_ratio: False
    pipeline:
      normalize: [[103.53, 116.28, 123.675], [57.375, 57.12, 58.395]]
device:
  gpu_ids: [0]
  workers_per_gpu: 2
  batchsize_per_gpu: 32
schedule:
  optimizer:
    name: AdamW
    lr: 0.001
    weight_decay: 0.05
  warmup:
    name: linear
    steps: 300
    ratio: 0.0001
  total_epochs: 200
  lr_schedule:
    name: CosineAnnealingLR
    T_max: 200
    eta_min: 0.00005
  val_intervals: 10
evaluator:
  name: CocoDetectionEvaluator
  save_key: mAP
log:
  interval: 50
"""

with open('/content/nanodet/config.yml', 'w') as f:
    f.write(config)
print('Config written (removed empty resume/load_model).')

# Downgrade pytorch-lightning to 1.x for nanodet compatibility
!pip install pytorch-lightning==1.9.5 -q 2>&1 | tail -3

# Train NanoDet-Plus
!python /content/nanodet-repo/tools/train.py /content/nanodet/config.yml

# Export to ONNX for Pi deployment
!python -m nanodet.tools.export_onnx --cfg_path /content/nanodet/config.yml --model_path /content/nanodet/workspace/model_best/model_best.ckpt --out_path /content/nanodet/nanodet_plus.onnx --input_shape 320 320

print('Exported to ONNX. Convert to TFLite INT8 for Pi deployment.')

# Export NanoDet model and save to Drive
!mkdir -p /content/nanodet/export
!cp /content/nanodet/nanodet_plus.onnx /content/nanodet/export/
!cp /content/nanodet/workspace/model_best/model_best.ckpt /content/nanodet/export/
!cp /content/nanodet/config.yml /content/nanodet/export/

!zip -r /content/nanodet/nanodet-cleanbot.zip /content/nanodet/export/
print('NanoDet export zipped. Download from /content/nanodet/nanodet-cleanbot.zip')

# zip export folder
%cd /content/cleanbot

!zip -r /content/cleanbot/cleanbot-yolo.zip /content/cleanbot/export/model
!zip -r /content/cleanbot/cleanbot-dataset.zip /content/cleanbot/export/dataset
!zip -r /content/cleanbot/cleanbot-train-logs.zip /content/cleanbot/export/train-logs
%cd /content
