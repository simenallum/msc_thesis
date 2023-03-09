import torch
import torch.nn.functional as F
from tqdm import tqdm
from unet import UNet
import argparse
import logging
from pathlib import Path
from utils.data_loading import BasicDataset
import os
from torch.utils.data import DataLoader

from utils.dice_score import multiclass_dice_coeff, dice_coeff


@torch.inference_mode()
def evaluate(net, dataloader, device, amp):
    net.eval()
    num_val_batches = len(dataloader)
    dice_score = 0
    img_ret = None
    mask_true_ret = None
    mask_pred_ret = None

    # iterate over the validation set
    with torch.autocast(device.type if device.type != 'mps' else 'cpu', enabled=amp):
        for batch in tqdm(dataloader, total=num_val_batches, desc='Validation round', unit='batch', leave=False):
            image, mask_true = batch['image'], batch['mask']

            # move images and labels to correct device and type
            image = image.to(device=device, dtype=torch.float32, memory_format=torch.channels_last)
            mask_true = mask_true.to(device=device, dtype=torch.long)

            # predict the mask
            mask_pred = net(image)

            if net.n_classes == 1:
                assert mask_true.min() >= 0 and mask_true.max() <= 1, 'True mask indices should be in [0, 1]'
                mask_pred = (torch.sigmoid(mask_pred.squeeze(1)) > 0.5).float()
                # compute the Dice score
                dice_score += dice_coeff(mask_pred, mask_true, reduce_batch_first=False)
            else:
                assert mask_true.min() >= 0 and mask_true.max() < net.n_classes, 'True mask indices should be in [0, n_classes['
                # convert to one-hot format
                mask_true = F.one_hot(mask_true, net.n_classes).permute(0, 3, 1, 2).float()
                mask_pred = F.one_hot(mask_pred.argmax(dim=1), net.n_classes).permute(0, 3, 1, 2).float()
                # compute the Dice score, ignoring background
                dice_score += multiclass_dice_coeff(mask_pred[:, 1:], mask_true[:, 1:], reduce_batch_first=False)

            img_ret = image
            mask_true_ret = mask_true
            mask_pred_ret = mask_pred

    net.train()
    return (dice_score / max(num_val_batches, 1)), img_ret, mask_true_ret, mask_pred_ret


def get_args():
    parser = argparse.ArgumentParser(description='Evaluate model on test data')    
    
    parser.add_argument('--model', '-m', default='MODEL.pth', metavar='FILE',
                        help='Specify the file in which the model is stored')
    
    return parser.parse_args()

if __name__ == '__main__':
    args = get_args()

    dir_img = Path('/home/simenallum/Desktop/SWED_images/test/images')
    dir_mask = Path('/home/simenallum/Desktop/SWED_images/test/labels')

    n_classes = 1
    bilinear = False
    img_scale = 0.5
    batch_size = 4

    net = UNet(n_channels=3, n_classes=n_classes, bilinear=bilinear)

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    net.to(device=device)
    state_dict = torch.load(args.model, map_location=device)
    mask_values = state_dict.pop('mask_values', [0, 1])
    net.load_state_dict(state_dict)

    logging.info('Model loaded!')

    dataset = BasicDataset(dir_img, dir_mask, img_scale)
    loader_args = dict(batch_size=batch_size, num_workers=os.cpu_count(), pin_memory=True)
    test_loader = DataLoader(dataset, shuffle=True, **loader_args)

    val_score = evaluate(net, test_loader, device, amp=True)

    print(val_score)


