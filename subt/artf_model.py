import cv2
import numpy as np
import random
import torch

ARTIFACT_CATEGORIES = dict((name, artf_id) for artf_id, name in enumerate([
    'nothing', 'backpack', 'phone', 'survivor', 'vent', 'robot', 'helmet',
    'rope', 'breadcrumb', 'drill', 'fire_extinguisher'
    #'toolbox', 'electrical_box', 'radio', 'valve'
]))


class SubtDataset(torch.utils.data.Dataset):
    def __init__(self,
                 labels,
                 img_root,
                 downscale,
                 receptive_field,
                 transform=None,
                 num_transforms=4,
                 a_few_negatives=200,
                 nothing_stride=10):
        import os
        import PIL

        super(SubtDataset, self).__init__()
        if transform is None:
            transform = torchvision.transforms.CenterCrop(receptive_field)
            num_transforms = 1

        self.patches = []
        self.artifacts = []
        self.origins = []

        blacklist = [
            ('backpack', '../virtual/backpack/backpack3.jpg'),  # ?
            ('backpack',
             '../virtual/backpack/rgbd/ji-ver54-saveX-0021.jpg'),  # ?
            ('backpack',
             '../virtual/survivor/rgbd/ji-ver54-saveX-0022.jpg'),  # ?
            ('backpack',
             '../virtual/survivor/rgbd/ji-ver54-saveX-0023.jpg'),  # ?
            ('phone', '../virtual/phone/saveX-0001.jpg'),
            ('phone', '../virtual/phone/saveX-0002.jpg'),
            ('phone', '../virtual/phone/saveX-0003.jpg'),
            ('phone', '../virtual/phone/saveX-0004.jpg'),
            ('phone', '../virtual/phone/saveX-0005.jpg'),
            ('phone', '../virtual/phone/saveX-0006.jpg'),
            ('phone', '../virtual/phone/saveX-0007.jpg'),
            ('phone', '../virtual/phone/saveX-0008.jpg'),
            ('phone', '../virtual/phone/saveX-0009.jpg'),
            ('phone', '../virtual/phone/saveX-0010.jpg'),
            ('phone', '../virtual/phone/saveX-0011.jpg'),
            ('phone', '../virtual/phone/saveX-0012.jpg'),
            ('phone', '../virtual/phone/saveX-0013.jpg'),
            ('phone', '../virtual/phone/saveX-0014.jpg'),
            ('phone', '../virtual/phone/saveX-0015.jpg'),
            ('phone', '../virtual/phone/saveX-0016.jpg'),
            ('phone', '../virtual/phone/saveX-0017.jpg'),
            ('phone', '../virtual/phone/saveX-0018.jpg'),
            ('phone', '../virtual/phone/saveX-0019.jpg'),
            ('phone', '../virtual/phone/saveX-0020.jpg'),
            ('phone', '../virtual/phone/saveX-0021.jpg'),
            ('phone', '../virtual/phone/saveX-0022.jpg'),
            ('phone', '../virtual/phone/saveX-0023.jpg'),
            ('phone', '../virtual/phone/saveX-0024.jpg'),
            ('phone', '../virtual/phone/saveX-0025.jpg'),
            ('phone', '../virtual/phone/saveX-0026.jpg'),
            ('phone', '../virtual/phone/saveX-0027.jpg'),
            ('phone', '../virtual/phone/saveX-0028.jpg'),
            ('phone', '../virtual/phone/saveX-0029.jpg'),
            ('phone', '../virtual/phone/saveX-0030.jpg'),
            ('phone', '../virtual/phone/saveX-0031.jpg'),
            ('phone', '../virtual/phone/saveX-0032.jpg'),
            #('phone', '../virtual/errors/ji-ver54-saveX-0014.jpg'),  # ?
            ('phone', '../virtual/errors/ji-ver54-saveX-0018.jpg'),  # ?
            ('survivor', '../virtual/survivor/005.jpg'),
            ('survivor', '../virtual/survivor/rgbd/ji-ver54-saveX-0023.jpg'),
            ('survivor', '../virtual/survivor/survivor-far.jpg')
        ]
        for label in labels:
            img_path = os.path.join(img_root, label['filename'])
            img = cv2.imread(img_path, cv2.IMREAD_COLOR)
            img = img[::downscale, ::downscale, :]
            h, w, _ = img.shape

            pts = []
            for region in label['regions']:
                shape = region['shape_attributes']
                x = shape['x'] // downscale
                y = shape['y'] // downscale
                width = shape['width'] // downscale
                height = shape['height'] // downscale
                artifact = region['region_attributes']['artifact']
                if artifact not in ARTIFACT_CATEGORIES:
                    continue
                if (artifact, label['filename']) in blacklist:
                    continue

                # Focus on the center of the artifact.
                pts.append((artifact, x + width / 2, y + height / 2))

                # Everything in a 'nothing' box is nothing.
                if artifact == 'nothing':
                    for v in range(y, y + height, nothing_stride):
                        for u in range(x, x + width, nothing_stride):
                            pts.append((artifact, u, v))

            # Add a few random points outside of bounding boxes as other
            # examples of 'nothing'.
            rnd_xs = np.random.default_rng().integers(0, img.shape[1],
                                                      a_few_negatives)
            rnd_ys = np.random.default_rng().integers(0, img.shape[0],
                                                      a_few_negatives)
            for (rnd_x, rnd_y) in zip(rnd_xs, rnd_ys):
                is_outside = True
                for region in label['regions']:
                    shape = region['shape_attributes']
                    x = shape['x'] // downscale
                    y = shape['y'] // downscale
                    width = shape['width'] // downscale
                    height = shape['height'] // downscale

                    if (x <= rnd_x < x + width) and (y <= rnd_y < y + height):
                        is_outside = False
                        break

                if is_outside:
                    pts.append(('nothing', rnd_x, rnd_y))

            for artifact, cx, cy in pts:
                # Giving ourselves some slack for transforms
                ltx = int(cx - receptive_field)
                lty = int(cy - receptive_field)
                if ltx < 0 or lty < 0:
                    continue
                rbx = ltx + receptive_field * 2
                rby = lty + receptive_field * 2
                if rbx > w or rby > h:
                    continue
                patch = img[lty:rby, ltx:rbx, :]
                assert (patch.shape == (receptive_field * 2,
                                        receptive_field * 2, 3))

                patch = PIL.Image.fromarray(
                    cv2.cvtColor(patch, cv2.COLOR_BGR2RGB))
                for _ in range(num_transforms):
                    patch_bgr = cv2.cvtColor(np.array(transform(patch)),
                                             cv2.COLOR_RGB2BGR)
                    self.patches.append(patch_bgr)
                    self.artifacts.append(artifact)
                    self.origins.append(label['filename'])

    def __len__(self):
        return len(self.patches)

    def __getitem__(self, idx):
        return (self.patches[idx], ARTIFACT_CATEGORIES[self.artifacts[idx]],
                self.origins[idx])

def __async_dataset_loader(output_queue, *args, **kwargs):
    output_queue.put(SubtDataset(*args, **kwargs))

def load_dataset(labels_config, workers=1,  *args, **kwargs):
    assert(workers > 0)
    import json
    import os

    with open(labels_config, 'r') as labels_file:
        labels = json.loads(labels_file.read())
    random.shuffle(labels)
    img_root = os.path.dirname(labels_config)

    if workers == 1:
        # No need for the overhead of running asynchronously.
        return SubtDataset(labels, img_root, *args, **kwargs)

    import multiprocessing
    sharded_labels = [[]] * workers
    for i, label in enumerate(labels.values()):
        sharded_labels[(i % workers)].append(label)
    result_queue = multiprocessing.Queue()
    loaders = [multiprocessing.Process(
        target=lambda: __async_dataset_loader(
            result_queue, label_shard, img_root, *args, **kwargs))
        for label_shard in sharded_labels]
    for i, loader in enumerate(loaders):
        print('Starting loader:', i)
        loader.start()
    partial_datasets = []
    for i in range(workers):
        print('Getting partial datasets:', i)
        partial_datasets.append(result_queue.get())
    for i, loader in enumerate(loaders):
        print('Waiting for loader:', i)
        loader.join()
        print('Loader', i, 'done.')
    return torch.utils.data.ChainDataset(partial_datasets)


class ChannelFirst(torch.nn.Module):
    def forward(self, x):
        # NHWC -> NCHW
        return x.permute(0, 3, 1, 2)


class ChannelLast(torch.nn.Module):
    def forward(self, x):
        # NCHW -> NHWC
        return x.permute(0, 2, 3, 1)


class Downscale(torch.nn.Module):
    def __init__(self, factor):
        super(Downscale, self).__init__()
        self.factor = factor

    def forward(self, x):
        return x[:, ::self.factor, ::self.factor, :]


class NormalizeImage(torch.nn.Module):
    def forward(self, x):
        return (x - 127.) / 128.


class Swish(torch.nn.Module):
    def forward(self, x):
        return x * torch.sigmoid(x)


class HSwish(torch.nn.Module):
    def forward(self, x):
        return x * torch.nn.functional.relu6(x + 3) / 6


class MaybeSoftMax(torch.nn.Module):
    def forward(self, x):
        if self.training:
            return x
        else:
            return torch.nn.functional.softmax(x, dim=1)


def activation_fn(fn):
    return {
        'relu': torch.nn.ReLU,
        'gelu': torch.nn.GELU,
        'swish': Swish,
        'hswish': HSwish
    }[fn]()


class MDNet0(torch.nn.Sequential):
    def __init__(self, downscale, embedding_size_0, embedding_size_1,
                 activation, dropout_probability, receptive_field,
                 num_categories):
        super(MDNet0, self).__init__(
            Downscale(downscale), NormalizeImage(), ChannelFirst(),
            torch.nn.Conv2d(3, embedding_size_0, kernel_size=1),
            torch.nn.BatchNorm2d(embedding_size_0), activation_fn(activation),
            torch.nn.Conv2d(embedding_size_0, embedding_size_0, kernel_size=1),
            torch.nn.BatchNorm2d(embedding_size_0), activation_fn(activation),
            torch.nn.Dropout2d(dropout_probability),
            torch.nn.Conv2d(embedding_size_0,
                            embedding_size_1,
                            kernel_size=receptive_field,
                            padding=receptive_field // 2),
            torch.nn.BatchNorm2d(embedding_size_1), activation_fn(activation),
            torch.nn.Conv2d(embedding_size_1, num_categories, kernel_size=1),
            MaybeSoftMax(), ChannelLast())


class MBConv(torch.nn.Module):
    ''' A block from MobileNet V2. '''
    def __init__(self,
                 input_channels,
                 expansion_factor,
                 activation,
                 kernel=3,
                 stride=1):
        super(MBConv, self).__init__()
        expanded_channels = expansion_factor * input_channels
        self.cconv0 = torch.nn.Conv2d(input_channels,
                                      expanded_channels,
                                      kernel_size=1)
        self.bn0 = torch.nn.BatchNorm2d(expanded_channels)
        self.dconv = torch.nn.Conv2d(expanded_channels,
                                     expanded_channels,
                                     kernel_size=kernel,
                                     padding=kernel // 2,
                                     groups=expanded_channels)
        self.bn1 = torch.nn.BatchNorm2d(expanded_channels)
        self.cconv1 = torch.nn.Conv2d(expanded_channels,
                                      input_channels,
                                      kernel_size=1)
        self.bn2 = torch.nn.BatchNorm2d(input_channels)
        self.activation = activation_fn(activation)

    def forward(self, x):
        x0 = x
        x = self.cconv0(x)
        x = self.bn0(x)
        x = self.activation(x)
        x - self.dconv(x)
        x = self.bn1(x)
        x = self.activation(x)
        x = self.cconv1(x)
        x = self.bn2(x)
        x += x0
        return x


class MBNet0(torch.nn.Sequential):
    def __init__(self, downscale, dropout_probability, embedding_size,
                 activation, convolution_layers, embedding_expansion_factor,
                 kernel_size, num_categories):
        conv_layers = [
            MBConv(embedding_size, embedding_expansion_factor, activation,
                   kernel_size) for _ in range(convolution_layers)
        ]
        super(MBNet0, self).__init__(
            Downscale(downscale), NormalizeImage(), ChannelFirst(),
            torch.nn.Dropout2d(dropout_probability),
            torch.nn.Conv2d(3, embedding_size, 3, 1, 3 // 2), *conv_layers,
            torch.nn.Conv2d(embedding_size, num_categories, kernel_size=1),
            MaybeSoftMax(), ChannelLast())


def create_model(model_config):
    # Other model architectures may be supported in future.
    assert (model_config['type'] in ['mdnet0', 'mbnet0'])
    if model_config['type'] == 'mdnet0':
        model = MDNet0(model_config['downscale'],
                       model_config['embedding-size-0'],
                       model_config['embedding-size-1'],
                       model_config['activation'],
                       model_config['dropout-probability'],
                       model_config['receptive-field'],
                       len(model_config['categories']))
    elif model_config['type'] == 'mbnet0':
        model = MBNet0(model_config['downscale'],
                       model_config['dropout-probability'],
                       model_config['embedding-size'],
                       model_config['activation'],
                       model_config['convolution-layers'],
                       model_config['embedding-expansion-factor'],
                       model_config['kernel-size'],
                       len(model_config['categories']))
    return model


def load_model(path, device):
    model_params = torch.load(path, map_location=torch.device(device))
    model_type = model_params.get('type', 'mdnet0')
    model = create_model(model_params)
    model = model.float().to(device)
    model.load_state_dict(model_params['model'])
    model.eval()

    return model, model_params['categories']


def save_model(model_config, model, path):
    # Other model architectures may be supported in future.
    assert (model_config['type'] in ['mdnet0', 'mbnet0'])
    if model_config['type'] == 'mdnet0':
        torch.save(
            {
                'type': 'mdnet0',
                'downscale': model_config['downscale'],
                'dropout-probability': model_config['dropout-probability'],
                'receptive-field': model_config['receptive-field'],
                'embedding-size-0': model_config['embedding-size-0'],
                'embedding-size-1': model_config['embedding-size-1'],
                'activation': model_config['activation'],
                'categories': model_config['categories'],
                'model': model.state_dict()
            }, path)
    elif model_config['type'] == 'mbnet0':
        torch.save(
            {
                'type': 'mbnet0',
                'downscale': model_config['downscale'],
                'dropout-probability': model_config['dropout-probability'],
                'embedding-size': model_config['embedding-size'],
                'activation': model_config['activation'],
                'convolution-layers': model_config['convolution-layers'],
                'embedding-expansion-factor':
                model_config['embedding-expansion-factor'],
                'kernel-size': model_config['kernel-size'],
                'categories':  model_config['categories'],
                'model': model.state_dict()
            }, path)


# Focal loss reduces weight of confidently correctly classified samples,
# thereby making training focus on samples where classification is not confident
# or even incorrect.
# https://arxiv.org/abs/1708.02002
# https://discuss.pytorch.org/t/is-this-a-correct-implementation-for-focal-loss-in-pytorch/43327/8
def focal_loss(prediction, target, weight=None, gamma=2., reduction='mean'):
    log_prob = torch.nn.functional.log_softmax(prediction, dim=-1)
    prob = torch.exp(log_prob)
    return torch.nn.functional.nll_loss(((1 - prob)**gamma) * log_prob,
                                        target,
                                        weight=weight,
                                        reduction=reduction)


def evaluate_model(model, batch_input, targets, receptive_field):
    predictions = model(batch_input)[:, receptive_field // 2,
                                     receptive_field // 2, :]
    #loss = torch.nn.functional.cross_entropy(predictions, targets)
    loss = focal_loss(predictions, targets)
    categories = torch.argmax(predictions, dim=1)
    is_correct = categories == targets

    return loss, is_correct


if __name__ == '__main__':
    import argparse
    import json
    import time
    import torchvision

    parser = argparse.ArgumentParser()
    parser.add_argument('--labels',
                        default=None,
                        help='Path to a json file with image annotations.')
    parser.add_argument('--downscale',
                        type=int,
                        default=4,
                        help='How much to shrink the input image.')
    parser.add_argument('--receptive-field',
                        type=int,
                        default=13,
                        help='Width (and height) of an area used for pixel ' +
                        'classification.')
    parser.add_argument('--model-config',
                        type=json.loads,
                        default={
                            "type": "mdnet0",
                            "embedding-size-0": 64,
                            "embedding-size-1": 64,
                            "activation": "relu",
                            "dropout-probability": 0.4
                        })
    parser.add_argument('--batch-size',
                        type=int,
                        default=1024,
                        help='Training batch size.')
    parser.add_argument('--num-loading-workers',
                        type=int,
                        default=4,
                        help='Training batch size.')
    parser.add_argument('--epochs',
                        type=int,
                        default=230,
                        help='Number of training epochs')
    parser.add_argument('--num-training-transforms',
                        type=int,
                        default=32,
                        help='Number for random transormations of training ' +
                        'patches.')
    parser.add_argument('--num-testing-transforms',
                        type=int,
                        default=1,
                        help='Number for random transormations of testing ' +
                        'patches.')
    parser.add_argument('--a-few-negatives',
                        type=int,
                        default=32,
                        help='Number of negatives taken per image.')
    parser.add_argument('--nothing-stride',
                        type=int,
                        default=10,
                        help='Density of pixels taken from `nothing` ' +
                        'training samples.')
    parser.add_argument('--lr',
                        type=float,
                        default=10.0,
                        help='Learning rate.')
    parser.add_argument('--gamma',
                        type=float,
                        default=0.999,
                        help='Learning rate step gamma.')
    parser.add_argument('--saved-model',
                        default=None,
                        help='Path to where to save the trained model.')
    parser.add_argument('--evaluation-frequency',
                        type=int,
                        default=10,
                        metavar='N',
                        help='Evaluation of the model done every N epochs.')
    args = parser.parse_args()

    use_cuda = torch.cuda.is_available()
    kwargs = {
        'num_workers': args.num_loading_workers,
        'pin_memory': False
    } if use_cuda else {}
    device = torch.device("cuda" if use_cuda else "cpu")
    print('Device:', device)
    torch.manual_seed(41)
    random.seed(41)

    image_transforms = torchvision.transforms.Compose([
        torchvision.transforms.RandomAffine(
            degrees=20,
            scale=(0.9, 1.1),
            translate=(1 / args.receptive_field, 1 / args.receptive_field)),
        torchvision.transforms.RandomHorizontalFlip(),
        torchvision.transforms.CenterCrop(args.receptive_field),
        torchvision.transforms.ColorJitter(brightness=(0.8, 1.2),
                                           contrast=(0.8, 1.2),
                                           saturation=(0.8, 1.2),
                                           hue=0.02)
    ])
    training_set = load_dataset(
        labels_config=args.labels,
        workers=args.num_loading_workers,
        downscale=args.downscale,
        receptive_field=args.receptive_field,
        transform=(image_transforms if args.num_testing_transforms else None),
        num_transforms=args.num_training_transforms,
        a_few_negatives=args.a_few_negatives,
        nothing_stride=args.nothing_stride)
    training_loader = torch.utils.data.DataLoader(training_set,
                                                  batch_size=args.batch_size,
                                                  **kwargs)
    print('Loaded traning set.')

    test_set = load_dataset(labels_config=args.labels,
                            workers=args.num_loading_workers,
                            downscale=args.downscale,
                            receptive_field=args.receptive_field,
                            transform=image_transforms,
                            num_transforms=args.num_testing_transforms,
                            a_few_negatives=args.a_few_negatives,
                            nothing_stride=args.nothing_stride)
    test_set = training_set
    test_loader = torch.utils.data.DataLoader(test_set,
                                              batch_size=args.batch_size,
                                              **kwargs)
    print('Loaded test set.')

    # No downscaling during training, because dataset loader already takes care
    # of that.
    args.model_config['downscale'] = 1
    args.model_config['receptive-field'] = args.receptive_field
    args.model_config['categories'] = ARTIFACT_CATEGORIES
    model = create_model(args.model_config)
    print('Created model.')

    model = model.float()
    if use_cuda:
        model = model.cuda()

    optimizer = torch.optim.Adadelta(model.parameters(), lr=args.lr)
    scheduler = torch.optim.lr_scheduler.StepLR(optimizer,
                                                step_size=1,
                                                gamma=args.gamma)

    best_accuracy = -1
    for epoch in range(1, args.epochs + 1):
        model.train()
        for batch_idx, (patches, targets,
                        origins) in enumerate(training_loader):
            patches = patches.float().to(device)
            targets = targets.to(device)
            optimizer.zero_grad()
            loss, is_correct = evaluate_model(model, patches, targets,
                                              args.receptive_field)
            accuracy = torch.mean(is_correct.float())
            loss.backward()
            optimizer.step()
            print('TRAIN epoch={}, batch={}, loss={}, accuracy={}'.format(
                epoch, batch_idx, loss.item(), accuracy.item()))

        if epoch % args.evaluation_frequency == 0:
            model.eval()
            n = 0
            total_acc = 0.0
            for batch_idx, (patches, targets,
                            origins) in enumerate(test_loader):
                patches = patches.float().to(device)
                targets = targets.to(device)
                loss, is_correct = evaluate_model(model, patches, targets,
                                                  args.receptive_field)
                total_acc += torch.sum(is_correct.float()).item()
                n += len(patches)
                from_category = dict(
                    (cid, cat) for (cat, cid) in ARTIFACT_CATEGORIES.items())
                for c, o, t in zip(is_correct, origins, targets):
                    if not c.item():
                        print('misclassified:', from_category[t.item()], o)

            acc = total_acc / n
            # Not aggregating and printing `loss`, because the output
            # MaybeSoftMax layer makes it incorrect. CrossEntropyLoss expects
            # logits, not probabilities.
            print('TEST epoch={}, accuracy={}'.format(epoch, acc))
            time.sleep(0.5)  # To gove myself time to spot the message above.
            if acc > best_accuracy:
                best_accuracy = acc
                if args.saved_model:
                    args.model_config['downscale'] = args.downscale
                    save_model(args.model_config, model, args.saved_model)
            if acc == 1.0:
                break

        scheduler.step()
