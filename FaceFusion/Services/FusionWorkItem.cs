/*
 * This file is part of the Face Fusion project. 
 *
 * Copyright (c) 2013 Joshua Blake
 *
 * This code is licensed to you under the terms of the MIT license.
 * See https://facefusion.codeplex.com/license for a copy of the license.
 */

using System;
using Blake.NUI.WPF.Utility;
using Microsoft.Kinect;

namespace FaceFusion.Services
{
    class FusionWorkItem : PoolItem<DepthImageFormat>
    {
        public DepthImagePixel[] Data { get; private set; }

        public FusionWorkItem(DepthImagePixel[] data, DepthImageFormat format)
            : base(format)
        {
            if (data == null)
            {
                throw new ArgumentNullException();
            }
            this.Data = data;
        }

        public static FusionWorkItem Create(DepthImageFormat depthFormat)
        {
            var size = FormatHelper.GetDepthSize(depthFormat);
            var data = new DepthImagePixel[(int)(size.Width * size.Height)];
            return new FusionWorkItem(data, depthFormat);
        }
    }        
}
