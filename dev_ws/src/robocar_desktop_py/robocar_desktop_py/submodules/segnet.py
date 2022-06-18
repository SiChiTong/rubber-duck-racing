import cv2
import torch
import segmentation_models_pytorch as smp
import albumentations as albu


class SegNet():
    def __init__(self, model_path, res=(384, 288), encoder='timm-mobilenetv3_small_minimal_100', encoder_weights='imagenet'):
        self.model = torch.load(model_path)
        self.res = res
        preprocessing_fn = smp.encoders.get_preprocessing_fn(encoder, encoder_weights)
        self.preprocessing  = self.get_preprocessing(preprocessing_fn)
        pass

    def to_tensor(self, x, **kwargs):
        return x.transpose(2, 0, 1).astype('float32')

    def get_preprocessing(self, preprocessing_fn):
        """Construct preprocessing transform
        
        Args:
            preprocessing_fn (callbale): data normalization function 
                (can be specific for each pretrained neural network)
        Return:
            transform: albumentations.Compose
        
        """
        
        _transform = [
            albu.Lambda(image=preprocessing_fn),
            albu.Lambda(image=self.to_tensor, mask=self.to_tensor),
        ]
        return albu.Compose(_transform)

    def detect_lines(self, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, self.res)
        # apply preprocessing
        sample = self.preprocessing(image=image)
        image = sample['image']

        x_tensor = torch.from_numpy(image).to('cuda').unsqueeze(0)
        pr_mask = self.model.predict(x_tensor)
        pr_mask = (pr_mask.squeeze().cpu().numpy().round())

        blue_mask = pr_mask[0]
        yellow_mask = pr_mask[1]
        return blue_mask, yellow_mask