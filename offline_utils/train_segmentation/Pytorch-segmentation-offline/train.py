import argparse
import logging
import os
import random
import sys
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.transforms as transforms
import torchvision.transforms.functional as TF
from pathlib import Path
from torch import optim
from torch.utils.data import DataLoader, random_split
from tqdm import tqdm

import wandb
from evaluate import evaluate
from unet import UNet
from segnet import SegNet_model

from utils.data_loading import BasicDataset, CarvanaDataset
from utils.dice_score import dice_loss
import datetime


dir_img = Path('/home/msccomputer/Desktop/segmentation_full_dataset/images')
dir_mask = Path('/home/msccomputer/Desktop/segmentation_full_dataset/labels')
dir_checkpoint = Path('/home/msccomputer/catkin_ws/src/msc_thesis/train_segmentation/checkpoints/')

dir_best_model = '/home/msccomputer/catkin_ws/src/msc_thesis/train_segmentation/BEST_MODELS/'
dir_best_model = Path(dir_best_model + datetime.datetime.now().strftime("%d-%m-%Y-%H-%M"))

def train_model(
		model,
		device,
		epochs: int = 5,
		batch_size: int = 1,
		learning_rate: float = 1e-5,
		val_percent: float = 0.2,
		save_checkpoint: bool = True,
		save_best_model: bool = True,
		img_scale: float = 0.5,
		amp: bool = False,
		gradient_clipping: float = 1.0,
		early_stopping_patience: int = 10
):
	# 1. Create dataset
	try:
		dataset = CarvanaDataset(dir_img, dir_mask, img_scale)
	except (AssertionError, RuntimeError, IndexError):
		dataset = BasicDataset(dir_img, dir_mask, img_scale)

	# 2. Split into train / validation partitions
	n_val = int(len(dataset) * val_percent)
	n_train = len(dataset) - n_val
	train_set, val_set = random_split(dataset, [n_train, n_val], generator=torch.Generator().manual_seed(1))
	

	# 3. Create data loaders
	loader_args = dict(batch_size=batch_size, num_workers=os.cpu_count(), pin_memory=True)
	train_loader = DataLoader(train_set, shuffle=True, **loader_args)
	val_loader = DataLoader(val_set, shuffle=True, drop_last=True, **loader_args)

	# (Initialize logging)
	experiment = wandb.init(project='Full-dataset', resume='allow', anonymous='must')
	experiment.config.update(
		dict(epochs=epochs, batch_size=batch_size, learning_rate=learning_rate,
			 val_percent=val_percent, save_checkpoint=save_checkpoint, img_scale=img_scale, amp=amp)
	)

	logging.info(f'''Starting training:
		Epochs:          {epochs}
		Batch size:      {batch_size}
		Learning rate:   {learning_rate}
		Training size:   {n_train}
		Validation size: {n_val}
		Checkpoints:     {save_checkpoint}
		Device:          {device.type}
		Images scaling:  {img_scale}
		Mixed Precision: {amp}
	''')

	# 4. Set up the optimizer, the loss, the learning rate scheduler and the loss scaling for AMP
	optimizer = optim.RMSprop(model.parameters(),
								lr=learning_rate, 
								weight_decay=0, 
								momentum=0, 
								foreach=True)
	
	# optimizer = optim.Adam(model.parameters(),
	# 							foreach=True)


	scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, 'max', patience=5)  # goal: maximize Dice score
	grad_scaler = torch.cuda.amp.GradScaler(enabled=amp)
	criterion = nn.CrossEntropyLoss() if model.n_classes > 1 else nn.BCEWithLogitsLoss()
	global_step = 0


	best_val_score = -1e3
	early_stopping_val_score = -1e3
	early_stopping_epochs_with_no_change = 0
	# 5. Begin training
	for epoch in range(1, epochs + 1):
		model.train()
		epoch_loss = 0
		with tqdm(total=n_train, desc=f'Epoch {epoch}/{epochs}', unit='img') as pbar:
			for batch in train_loader:
				images, true_masks = batch['image'], batch['mask']

				assert images.shape[1] == model.n_channels, \
				f'Network has been defined with {model.n_channels} input channels, ' \
				f'but loaded images have {images.shape[1]} channels. Please check that ' \
				'the images are loaded correctly.'

				images = images.to(device=device, dtype=torch.float32, memory_format=torch.channels_last)
				true_masks = true_masks.to(device=device, dtype=torch.long)

				# with torch.autocast(device.type if device.type != 'mps' else 'cpu', enabled=amp):
				with torch.autocast(device_type='cuda', enabled=amp):

					masks_pred = model(images)
					if model.n_classes == 1:
						loss = criterion(masks_pred.squeeze(1), true_masks.float())
						d_loss = dice_loss(torch.sigmoid(masks_pred.squeeze(1)), true_masks.float(), multiclass=False)
						loss += d_loss
					else:
						loss = criterion(masks_pred, true_masks)
						d_loss = dice_loss(
							F.softmax(masks_pred, dim=1).float(),
							F.one_hot(true_masks, model.n_classes).permute(0, 3, 1, 2).float(), multiclass=True)
						loss += d_loss

				optimizer.zero_grad(set_to_none=True)
				grad_scaler.scale(loss).backward()
				torch.nn.utils.clip_grad_norm_(model.parameters(), gradient_clipping)
				grad_scaler.step(optimizer)
				grad_scaler.update()

				pbar.update(images.shape[0])
				global_step += 1
				epoch_loss += loss.item()
				experiment.log({
					'train loss': loss.item(),
					'train Dice': d_loss.item() / batch_size,
					'step': global_step,
					'epoch': epoch
				})
				pbar.set_postfix(**{'loss (batch)': loss.item()})

		experiment.log({
					'Epoch train loss': epoch_loss,
					'epoch': epoch
				})
		# 6. Evaluate model on validation dataset
		val_score, val_img, val_mask_true, val_mask_pred = evaluate(model, val_loader, device, amp)
		scheduler.step(val_score)

		logging.info('Validation Dice score: {}'.format(val_score))
		try:
			if model.n_classes > 1:
				mask = val_mask_pred.argmax(dim=1)
			else:
				mask = torch.sigmoid(val_mask_pred) > 0.5

			experiment.log({
				'learning rate': optimizer.param_groups[0]['lr'],
				'validation Dice': val_score,
				'images': wandb.Image(val_img[0].cpu()),
				'masks': {
					'true': wandb.Image(val_mask_true[0].float().cpu()),
					'pred': wandb.Image(mask[0].float().cpu()),
				},
				'step': global_step,
				'epoch': epoch
			})
				
		except Exception as e:
			print(e)
			pass

		if save_best_model:
			if val_score > best_val_score:
				Path(dir_best_model).mkdir(parents=True, exist_ok=True)
				state_dict = model.state_dict()
				state_dict['mask_values'] = dataset.mask_values
				torch.save(state_dict, str(dir_best_model / 'best_model.pth'))
				logging.info(f'New best model after epoch number: {epoch}!')
				logging.info(f'The saved model has a validation score of: {val_score:.6f}')
				best_val_score = val_score

		if save_checkpoint:
			Path(dir_checkpoint).mkdir(parents=True, exist_ok=True)
			state_dict = model.state_dict()
			state_dict['mask_values'] = dataset.mask_values
			torch.save(state_dict, str(dir_checkpoint / 'checkpoint.pth'))
			logging.info(f'Checkpoint {epoch} saved!')

		# Epoch score has to be better than a improvement of 0.5% of the old best score
		val_mult_score = val_score * 1.005
		if  val_mult_score > early_stopping_val_score:
			print(f"EARLY STOPPING RESET! 0.5% improvement of old score: {val_mult_score}. Old score: {early_stopping_val_score}")
			early_stopping_epochs_with_no_change = 0
			if val_score > early_stopping_val_score:
				early_stopping_val_score = val_score
		else:
			early_stopping_epochs_with_no_change += 1
			print(f"0.5% improvement of old score: {val_mult_score}. Old score: {early_stopping_val_score}")
			print(f"Early stopping epochs with no change: {early_stopping_epochs_with_no_change}")


		if early_stopping_epochs_with_no_change >= early_stopping_patience:
			logging.info(f'Early stopping after epoch: {epoch}!')
			return
		
		print("\n")


def get_args():
	parser = argparse.ArgumentParser(description='Train the UNet on images and target masks')
	parser.add_argument('--epochs', '-e', metavar='E', type=int, default=5, help='Number of epochs')
	parser.add_argument('--batch-size', '-b', dest='batch_size', metavar='B', type=int, default=1, help='Batch size')
	parser.add_argument('--learning-rate', '-l', metavar='LR', type=float, default=1e-5,
						help='Learning rate', dest='lr')
	parser.add_argument('--load', '-f', type=str, default=False, help='Load model from a .pth file')
	parser.add_argument('--scale', '-s', type=float, default=0.5, help='Downscaling factor of the images')
	parser.add_argument('--validation', '-v', dest='val', type=float, default=10.0,
						help='Percent of the data that is used as validation (0-100)')
	parser.add_argument('--amp', action='store_true', default=False, help='Use mixed precision')
	parser.add_argument('--bilinear', action='store_true', default=False, help='Use bilinear upsampling')
	parser.add_argument('--classes', '-c', type=int, default=2, help='Number of classes')
	parser.add_argument('--model_type', type=str, default="", help='Modeltype: "unet" or "segnet"')

	return parser.parse_args()


if __name__ == '__main__':
	args = get_args()

	logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
	device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
	logging.info(f'Using device {device}')

	# Change here to adapt to your data
	# n_channels=3 for RGB images
	# n_classes is the number of probabilities you want to get per pixel
	if args.model_type == "unet":
		logging.info(f'Network typ: UNet\n')
		model = UNet(n_channels=3, n_classes=args.classes, bilinear=args.bilinear)
	elif args.model_type == "segnet":
		logging.info(f'Network typ: SegNet\n')
		model = SegNet_model.SegNet(n_channels=3, n_classes=args.classes)
	else:
		logging.warning(f"Unknown model type {args.model_type}. Exiting!")
		exit()
	
	model = model.to(memory_format=torch.channels_last)

	logging.info(f'Network:\n'
				 f'\t{model.n_channels} input channels\n'
				 f'\t{model.n_classes} output channels (classes)\n')

	if args.model_type == "unet":
		logging.info(f'{"Bilinear" if model.bilinear else "Transposed conv"} upscaling')

	if args.load:
		state_dict = torch.load(args.load, map_location=device)
		del state_dict['mask_values']
		model.load_state_dict(state_dict)
		logging.info(f'Model loaded from {args.load}')

	model.to(device=device)
	try:
		train_model(
			model=model,
			epochs=args.epochs,
			batch_size=args.batch_size,
			learning_rate=args.lr,
			device=device,
			img_scale=args.scale,
			val_percent=args.val / 100,
			amp=args.amp
		)
	except torch.cuda.OutOfMemoryError:
		logging.error('Detected OutOfMemoryError! '
					  'Enabling checkpointing to reduce memory usage, but this slows down training. '
					  'Consider enabling AMP (--amp) for fast and memory efficient training')
		torch.cuda.empty_cache()
		model.use_checkpointing()
		train_model(
			model=model,
			epochs=args.epochs,
			batch_size=args.batch_size,
			learning_rate=args.lr,
			device=device,
			img_scale=args.scale,
			val_percent=args.val / 100,
			amp=args.amp
		)

