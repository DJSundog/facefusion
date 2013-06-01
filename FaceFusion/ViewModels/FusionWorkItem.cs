using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Blake.NUI.WPF.Utility;
using Microsoft.Kinect;

namespace FaceFusion.ViewModels
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
