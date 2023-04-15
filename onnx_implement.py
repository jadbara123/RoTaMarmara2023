import cv2

# Load Model
net = cv2.dnn.readNet('/home/can/best.onnx')

# Load Model
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
INPUT_WIDTH = 640
INPUT_HEIGHT = 640
SCORE_THRESHOLD = 0.2
NMS_THRESHOLD = 0.4
CONFIDENCE_THRESHOLD = 0.4

# Define yolov8 classes
CLASESS_YOLO = [
 '20_hiz_siniri', '30_hiz_siniri', 'dur','park_yapilmaz', 'saga_donulmez', 'sola_donulmez', 'arac_gecisine_yasak_yol', 'trafik_isigi', 'ileri_ve_sola_mecburi_yon', ' ', 'kapali_yol', 'siz_siniri_sonu', 'park_yasak', ' ', 'ileriden_saga_mecburi_yon', 'park'  
]
image = cv2.imread('/home/can/img.jpg')
blob = cv2.dnn.blobFromImage(image, 1/255.0, (INPUT_WIDTH, INPUT_HEIGHT), swapRB=True, crop=False)
net.setInput(blob)
preds = net.forward()
preds = preds.transpose((0, 2, 1))
# Extract output detection
class_ids, confs, boxes = list(), list(), list()

image_height, image_width, _ = imgae.shape
x_factor = image_width / INPUT_WIDTH
y_factor = image_height / INPUT_HEIGHT

rows = preds[0].shape[0]

for i in range(rows):
    row = preds[0][i]
    conf = row[4]
    
    classes_score = row[4:]
    _,_,_, max_idx = cv2.minMaxLoc(classes_score)
    class_id = max_idx[1]
    if (classes_score[class_id] > .25):
        confs.append(conf)
        label = CLASESS_YOLO[int(class_id)]
        class_ids.append(label)
        
        #extract boxes
        x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() 
        left = int((x - 0.5 * w) * x_factor)
        top = int((y - 0.5 * h) * y_factor)
        width = int(w * x_factor)
        height = int(h * y_factor)
        box = np.array([left, top, width, height])
        boxes.append(box)
        
r_class_ids, r_confs, r_boxes = list(), list(), list()

indexes = cv2.dnn.NMSBoxes(boxes, confs, 0.25, 0.45) 
for i in indexes:
    r_class_ids.append(class_ids[i])
    r_confs.append(confs[i])
    r_boxes.append(boxes[i])
    for i in indexes:
    	box = boxes[i]
    	left = box[0]
    	top = box[1]
    	width = box[2]
    	height = box[3]
    
    cv2.rectangle(image, (left, top), (left + width, top + height), (0,255,0), 3)
Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
