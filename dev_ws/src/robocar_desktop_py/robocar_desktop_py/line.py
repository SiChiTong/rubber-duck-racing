import cv2
import torch
import segmentation_models_pytorch as smp
import numpy as np
from util.util import get_preprocessing

# load best model from file
best_model = torch.load('./best_model.pth')
# initiate the camera
cap = cv2.VideoCapture(0)

# specify the encoder and find the appropriate preprocessing for the image
ENCODER = 'timm-mobilenetv3_small_minimal_100'
ENCODER_WEIGHTS = 'imagenet'
preprocessing_fn = smp.encoders.get_preprocessing_fn(ENCODER, ENCODER_WEIGHTS)
preprocessing  = get_preprocessing(preprocessing_fn)

midX = 0
yellowX = 0
blueX = 0
blueRet = False
yellowRet = False

imsizeX = 128
imsizeY = 96
predictionHeight = imsizeY//8

while(1):
    # read the frame
    ret, f = cap.read()
    if (ret):
        # convert to RGB and resize the image
        image = cv2.cvtColor(f, cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, (imsizeX, imsizeY))

        # apply preprocessinga
        sample = preprocessing(image=image)
        image = sample['image']
        x_tensor = torch.from_numpy(image).to('cuda').unsqueeze(0)

        # make prediction, undo the processing and combine
        pr_mask = best_model.predict(x_tensor)
        pr_mask = (pr_mask.squeeze().cpu().numpy().round())
        predictionYellow = pr_mask[0][imsizeY-predictionHeight:imsizeY]
        predictionBlue = pr_mask[1][imsizeY-predictionHeight:imsizeY]

        # create an empty array for drawing
        prediction = np.zeros((predictionHeight, imsizeX, 3), dtype=float)

        # detect, filter, and draw contours
        minimumArea = 15
        yellowCont, yellowHier = cv2.findContours(predictionYellow.astype(np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        if len(yellowCont) > 0:
            yellowRet = True
            cont = max(yellowCont, key = cv2.contourArea)
            M = cv2.moments(cont)
            try:
                yellowX = int(M['m10']/M['m00'])
            except:
                yellowRet = False
            cv2.drawContours(prediction, cont, -1, (0, 0.5, 0.5), -1)
        else:
            yellowRet = False

        blueCont, blueHier = cv2.findContours(predictionBlue.astype(np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        if len(blueCont) > 0:
            blueRet = True
            cont = max(blueCont, key = cv2.contourArea)
            M = cv2.moments(cont)
            try:
                blueX = int(M['m10']/M['m00'])
            except:
                blueRet = False
            cv2.drawContours(prediction, cont, -1, (0.5, 0, 0), -1)
        else:
            blueRet = False

        cv2.line(prediction, (imsizeX//2, 0), (imsizeX//2, imsizeY), (0, 0, 1), 1)

        if (yellowRet and blueRet):
            midX = (yellowX+blueX)//2
            cv2.circle(prediction, (yellowX, predictionHeight//2), 3, (0, 1, 1), 1)
            cv2.circle(prediction, (blueX, predictionHeight//2), 3, (1, 0, 0), 1)
            cv2.circle(prediction, (midX, predictionHeight//2), 3, (1, 1, 1), 1)
        elif (yellowRet):
            midX = yellowX + (imsizeX//3)
            cv2.circle(prediction, (yellowX, predictionHeight//2), 3, (0, 1, 1), 1)            
            cv2.circle(prediction, (midX, predictionHeight//2), 3, (1, 1, 1), 1)
        elif (blueRet):
            midX = blueX - (imsizeX//3)
            cv2.circle(prediction, (blueX, predictionHeight//2), 3, (1, 0, 0), 1)
            cv2.circle(prediction, (midX, predictionHeight//2), 3, (1, 1, 1), 1)


        # resize to the original size
        predictionYellow = cv2.resize(predictionYellow, (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))//8))
        predictionBlue = cv2.resize(predictionBlue, (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))//8))
        prediction = cv2.resize(prediction, (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))//8))

        # show the results
        cv2.imshow("frame", f)
        cv2.imshow("prediction", prediction)
        cv2.waitKey(1)