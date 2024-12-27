"""
    PointNetCVAE类
    用于条件变分自编码器的类构建
"""
import copy
import torch
import torch.nn as nn
import torch.utils.data
import math
import numpy as np


class PointNetEncoder(nn.Module):
    """
        点云编码器Encoder（负责将输入的点云，包括点的坐标和接触图值）编码为潜在空间表示（编码为特征值）

        class member：
            layers_size: 各卷积层的通道数配置
            conv_layers: 卷积层列表
            bn_layers: 批归一化层列表
            activate_func: 激活函数，使用ReLU
        
        class funtion:
            forward(x): 将输入的点云通过一系列卷积层和批归一化层，最后通过全局最大池化得到固定维度的特征表示
    """
    def __init__(self,
                 layers_size=[4, 64, 128, 512]):
        super(PointNetEncoder, self).__init__()
        self.layers_size = layers_size
        self.conv_layers = nn.ModuleList()
        self.bn_layers = nn.ModuleList()
        self.activate_func = nn.ReLU()

        for i in range(len(layers_size) - 1):
            self.conv_layers.append(nn.Conv1d(layers_size[i], layers_size[i + 1], 1))
            self.bn_layers.append(nn.BatchNorm1d(layers_size[i+1]))
            nn.init.xavier_normal_(self.conv_layers[-1].weight)

    def forward(self, x):
        # input: B * N * 4  | 'B'个点云，每个点云有'N'个点，每个点有4个特征
        # output: B * latent_size | 输出特征向量
        x = x.transpose(1, 2)
        for i in range(len(self.conv_layers) - 1):
            x = self.conv_layers[i](x)
            x = self.bn_layers[i](x)
            x = self.activate_func(x)
        x = self.bn_layers[-1](self.conv_layers[-1](x))
        x = torch.max(x, 2, keepdim=True)[0]
        x = x.view(-1, self.layers_size[-1])
        return x


class PointNetDecoder(nn.Module):
    """
        点云解码器（Decoder）负责将潜在空间的表示和点云特征解码为接触图值

        class member：
            global_feat_size: 全局特征的维度大小
            latent_size: 潜在空间的维度大小
            pointwise_layers_size: 对每个点分别进行卷积的卷积层配置
            global_layers_size: 全局特征卷积层的配置
            decoder_layers_size: 解码器最终阶段的卷积层配置
            pointwise_conv_layers：对每个点进行卷积的卷积层列表
            pointwise_bn_layers：对每个点进行卷积的批归一化层列表
            global_conv_layers：全局特征的卷积层列表
            global_bn_layers：全局特征的批归一化层列表
            activate_func：激活函数，使用ReLU
            sigmoid：解码器的输出使用Sigmoid函数进行归一化

        class funtion:
            forward(x, z_latent_code): 解码器的前向传播，先将点的特征通过一系列卷积层处理，然后与潜在空间变量结合，再经过解码器生成最终的接触图值
    """
    def __init__(self,
                 global_feat_size=512,
                 latent_size=128,
                 pointwise_layers_size=[3, 64, 64],
                 global_layers_size=[64, 128, 512],
                 decoder_layers_size=[64+512+128, 512, 64, 64, 1]):
        super(PointNetDecoder, self).__init__()
        assert global_feat_size == global_layers_size[-1]
        assert decoder_layers_size[0] == latent_size + global_feat_size + pointwise_layers_size[-1]

        self.global_feat_size = global_feat_size
        self.latent_size = latent_size
        self.pointwise_layers_size = pointwise_layers_size
        self.global_layers_size = global_layers_size
        self.decoder_layers_size = decoder_layers_size

        self.pointwise_conv_layers = nn.ModuleList()
        self.pointwise_bn_layers = nn.ModuleList()
        self.global_conv_layers = nn.ModuleList()
        self.global_bn_layers = nn.ModuleList()
        self.activate_func = nn.ReLU()

        for i in range(len(pointwise_layers_size) - 1):
            self.pointwise_conv_layers.append(nn.Conv1d(pointwise_layers_size[i], pointwise_layers_size[i + 1], 1))
            self.pointwise_bn_layers.append(nn.BatchNorm1d(pointwise_layers_size[i+1]))
            nn.init.xavier_normal_(self.pointwise_conv_layers[-1].weight)
        for i in range(len(global_layers_size) - 1):
            self.global_conv_layers.append(nn.Conv1d(global_layers_size[i], global_layers_size[i + 1], 1))
            self.global_bn_layers.append(nn.BatchNorm1d(global_layers_size[i+1]))
            nn.init.xavier_normal_(self.global_conv_layers[-1].weight)

        self.decoder_conv_layers = nn.ModuleList()
        self.decoder_bn_layers = nn.ModuleList()
        self.sigmoid = nn.Sigmoid()

        for i in range(len(decoder_layers_size) - 1):
            self.decoder_conv_layers.append(nn.Conv1d(decoder_layers_size[i], decoder_layers_size[i + 1], 1))
            self.decoder_bn_layers.append(nn.BatchNorm1d(decoder_layers_size[i + 1]))
            nn.init.xavier_normal_(self.decoder_conv_layers[-1].weight)

    def forward(self, x, z_latent_code):
        """
        :param x: B x N x 3
        :param z_latent_code: B x latent_size
        :return:

        输入数据：输入带有3D坐标和接触图值的点云数据
        编码：通过PointNetEncoder编码为潜在空间表示（均值和对数方差
        重参数化：通过reparameterize方法，使用均值和对数方差生成潜在空间变量
        解码：通过PointNetDecoder将潜在空间变量与点云特征解码为接触图值
        """
        bs = x.shape[0]
        npts = x.shape[1]

        pointwise_feature = x.transpose(1, 2)
        for i in range(len(self.pointwise_conv_layers) - 1):
            pointwise_feature = self.pointwise_conv_layers[i](pointwise_feature)
            pointwise_feature = self.pointwise_bn_layers[i](pointwise_feature)
            pointwise_feature = self.activate_func(pointwise_feature)
        pointwise_feature = self.pointwise_bn_layers[-1](self.pointwise_conv_layers[-1](pointwise_feature))

        global_feature = pointwise_feature.clone()
        for i in range(len(self.global_conv_layers) - 1):
            global_feature = self.global_conv_layers[i](global_feature)
            global_feature = self.global_bn_layers[i](global_feature)
            global_feature = self.activate_func(global_feature)
        global_feature = self.global_bn_layers[-1](self.global_conv_layers[-1](global_feature))
        global_feature = torch.max(global_feature, 2, keepdim=True)[0]
        global_feature = global_feature.view(bs, self.global_feat_size)

        global_feature = torch.cat([global_feature, z_latent_code], dim=1)
        global_feature = global_feature.view(bs, self.global_feat_size + self.latent_size, 1).repeat(1, 1, npts)
        pointwise_feature = torch.cat([pointwise_feature, global_feature], dim=1)
        for i in range(len(self.decoder_conv_layers) - 1):
            pointwise_feature = self.decoder_conv_layers[i](pointwise_feature)
            pointwise_feature = self.decoder_bn_layers[i](pointwise_feature)
            pointwise_feature = self.activate_func(pointwise_feature)
        pointwise_feature = self.decoder_bn_layers[-1](self.decoder_conv_layers[-1](pointwise_feature))

        # pointwise_feature: B x N x 1
        pointwise_feature = self.sigmoid(pointwise_feature).view(bs, npts)
        return pointwise_feature


# class MLP(nn.Module):
#     def __init__(self, layers=[512, 128, 128]):
#         super(MLP, self).__init__()
#         self.layers = layers
#         self.linear_layers = nn.ModuleList()
#         self.activate_func = nn.ReLU()
#         for i in range(len(layers) - 1):
#             self.linear_layers.append(nn.Linear(layers[i], layers[i+1]))
#
#     def forward(self, x):
#         for i in range(len(self.linear_layers) - 1):
#             x = self.linear_layers[i](x)
#             x = self.activate_func(x)
#         x = self.linear_layers[-1](x)
#         # if self.is_sigmoid_output:
#         #     x = torch.sigmoid(x)
#         return x


class PointNetCVAE(nn.Module):
    """
        整个CVAE模型的主体，它包含了编码器（encoder）、解码器（decoder）和用于生成潜在空间变量的模块

        class members:
            latent_size: 潜在空间的维度大小，默认为128
            encoder: 一个PointNetEncoder实例，用于将输入的点云编码为潜在空间的表示
            decoder: 一个PointNetDecoder实例，用于从潜在空间的表示和点云特征解码出接触图值（contact map values）
            encoder_z_means: 一个线性层，用于从编码器的输出中生成潜在空间的均值
            ncoder_z_logvars: 一个线性层，用于从编码器的输出中生成潜在空间的对数方差
        
        class funtion:
            forward(object_cmap): 完成前向传播，将输入的点云编码为潜在空间表示，并解码为接触图值。返回值包括接触图值、潜在空间的均值、对数方差和潜在空间变量
            inference(object_pts, z_latent_code): 仅执行解码步骤，给定点云特征和潜在空间变量，生成接触图值
            reparameterize(means, logvars): 使用重参数化技巧生成潜在空间变量，以便实现可导的采样操作
            forward_encoder(object_cmap): 编码器的前向传播，将输入的点云编码为潜在空间的均值和对数方差
            forward_decoder(object_pts, z_latent_code): 解码器的前向传播，将潜在空间变量解码为接触图值
    """
    def __init__(self,
                 latent_size=128,
                 encoder_layers_size=[4, 64, 128, 512],

                 decoder_global_feat_size=512,
                 decoder_pointwise_layers_size=[3, 64, 64],
                 decoder_global_layers_size=[64, 128, 512],
                 decoder_decoder_layers_size=[64+512+128, 512, 64, 64, 1]):
        super(PointNetCVAE, self).__init__()
        self.latent_size = latent_size
        self.encoder = PointNetEncoder(layers_size=encoder_layers_size)
        self.decoder = PointNetDecoder(latent_size=latent_size,
                                       global_feat_size=decoder_global_feat_size,
                                       pointwise_layers_size=decoder_pointwise_layers_size,
                                       global_layers_size=decoder_global_layers_size,
                                       decoder_layers_size=decoder_decoder_layers_size)

        self.encoder_z_means = nn.Linear(encoder_layers_size[-1], latent_size)
        self.encoder_z_logvars = nn.Linear(encoder_layers_size[-1], latent_size)

    def forward(self, object_cmap):
        """
        :param object_cmap: B x N x 4
        :return:
        """
        bs = object_cmap.shape[0]
        npts = object_cmap.shape[1]
        object_pts = object_cmap[:, :, :3].clone()
        means, logvars = self.forward_encoder(object_cmap=object_cmap)
        z_latent_code = self.reparameterize(means=means, logvars=logvars)
        cmap_values = self.forward_decoder(object_cmap[:, :, :3], z_latent_code).view(bs, npts)
        return cmap_values, means, logvars, z_latent_code

    def inference(self, object_pts, z_latent_code):
        """
        :param object_pts: B x N x 3
        :param z_latent_code: B x latent_size
        :return:
        """
        cmap_values = self.forward_decoder(object_pts, z_latent_code)
        return cmap_values

    def reparameterize(self, means, logvars):
        std = torch.exp(0.5 * logvars)
        eps = torch.randn_like(std)
        return means + eps * std

    def forward_encoder(self, object_cmap):
        cmap_feat = self.encoder(object_cmap)
        means = self.encoder_z_means(cmap_feat)
        logvars = self.encoder_z_logvars(cmap_feat)
        return means, logvars

    def forward_decoder(self, object_pts, z_latent_code):
        """
        :param object_pts: B x N x 3
        :param z_latent_code: B x latent_size
        :return:
        """
        cmap_values = self.decoder(object_pts, z_latent_code)
        return cmap_values


if __name__ == '__main__':
    device = 'cuda'
    mode = 'test'
    model = PointNetCVAE()

    if mode == 'train':
        bs = 4
        model.train().cuda()
        dummy_object_camp = torch.randn(bs, 2048, 4, device=device).float()
        dummy_cmap_values, dummy_means, dummy_logvars, dummy_z = model(dummy_object_camp)

        loss_recons = torch.square(dummy_cmap_values - dummy_object_camp[:, :, 3]).mean()
        print(f'recons mean square error: {loss_recons}')
    elif mode == 'test':
        bs = 4
        model.eval().cuda()
        dummy_object_pts = torch.randn(bs, 2048, 3, device=device).float()
        z = torch.randn(bs, model.latent_size, device=device).float()
        dummy_cmap_values = model.inference(dummy_object_pts, z)
        print(f'dummy output size: {dummy_cmap_values.size()}')
        #
    else:
        raise NotImplementedError()


