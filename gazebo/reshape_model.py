
import torch
import torch.nn

#
# reshape the model for N classes
#
def reshape_model(model, arch, num_classes):
	"""Reshape a model's output layers for the given number of classes"""

	# reshape output layers for the dataset
	if arch.startswith("resnet"):
		model.fc = torch.nn.Linear(model.fc.in_features, num_classes)
		print("=> reshaped ResNet fully-connected layer with: " + str(model.fc))

	elif arch.startswith("alexnet"):
		model.classifier[6] = torch.nn.Linear(model.classifier[6].in_features, num_classes)
		print("=> reshaped AlexNet classifier layer with: " + str(model.classifier[6]))

	elif arch.startswith("vgg"):
		model.classifier[6] = torch.nn.Linear(model.classifier[6].in_features, num_classes)
		print("=> reshaped VGG classifier layer with: " + str(model.classifier[6]))

	elif arch.startswith("squeezenet"):
		model.classifier[1] = torch.nn.Conv2d(512, num_classes, kernel_size=(1,1), stride=(1,1))
		model.num_classes = num_classes
		print("=> reshaped SqueezeNet classifier layer with: " + str(model.classifier[1]))

	elif arch.startswith("densenet"):
		model.classifier = torch.nn.Linear(model.classifier.in_features, num_classes) 
		print("=> reshaped DenseNet classifier layer with: " + str(model.classifier))

	elif arch.startswith("inception"):
		model.AuxLogits.fc = torch.nn.Linear(model.AuxLogits.fc.in_features, num_classes)
		model.fc = torch.nn.Linear(model.fc.in_features, num_classes)

		print("=> reshaped Inception aux-logits layer with: " + str(model.AuxLogits.fc))
		print("=> reshaped Inception fully-connected layer with: " + str(model.fc))
	
	elif arch.startswith("googlenet"):
		if model.aux_logits:
			from torchvision.models.googlenet import InceptionAux

			model.aux1 = InceptionAux(512, num_classes)
			model.aux2 = InceptionAux(528, num_classes)

			print("=> reshaped GoogleNet aux-logits layers with: ")
			print("      " + str(model.aux1))
			print("      " + str(model.aux2))
	
		model.fc = torch.nn.Linear(model.fc.in_features, num_classes)
		print("=> reshaped GoogleNet fully-connected layer with:  " + str(model.fc))
	
	else:
		print("classifier reshaping not supported for " + args.arch)
		print("model will retain default of 1000 output classes")

	return model

