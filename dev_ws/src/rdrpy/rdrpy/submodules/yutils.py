from .submodules.utils.general import non_max_suppression
from .submodules.models.experimental import attempt_load
import numpy as np
import torch
import cv2

class YUtils():
    def __init__(self):
        self.classes = ['Right', 'Left']
        self.object_colors = list(np.random.rand(80,3)*255)
        self.input_width = 320
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.yolo_model = attempt_load(weights="/home/jared/rubber-duck-racing/dev_ws/src/rdrpy/rdrpy/best.pt").to(self.device)

    def detect(self, image):
        height, width = image.shape[:2]
        new_height = int((((self.input_width/width)*height)//32)*32)

        img = cv2.resize(image, (self.input_width,new_height))
        img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
        img = np.moveaxis(img,-1,0)
        img = torch.from_numpy(img).to(self.device)
        img = img.float()/255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        pred = self.yolo_model(img, augment=False)[0]
        pred = non_max_suppression(pred, conf_thres=0.25, iou_thres=0.45, classes=None)
        
        if pred[0] is not None and len(pred):
            for p in pred[0]:
                score = np.round(p[4].cpu().detach().numpy(),2)
                if(score >= 0.90):
                    
                    label = self.classes[int(p[5])]
                    xmin = int(p[0] * image.shape[1] /self.input_width)
                    ymin = int(p[1] * image.shape[0] /new_height)
                    xmax = int(p[2] * image.shape[1] /self.input_width)
                    ymax = int(p[3] * image.shape[0] /new_height)
                    color = self.object_colors[self.classes.index(label)]
                    image = cv2.rectangle(image, (xmin,ymin), (xmax,ymax), color, 2) 
                    image = cv2.putText(image, f'{label} ({str(score)})', (xmin,ymin), cv2.FONT_HERSHEY_SIMPLEX , 0.75, color, 1, cv2.LINE_AA)
                    return int(p[5])+1
        return 0

